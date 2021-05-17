#include "world_renderer.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

WorldRenderer::WorldRenderer(QWidget* parent, const CameraIntrinsics& c)
    : QOpenGLWidget(parent), camera(c),
    map_type(MapType::AUTO),
    cloud_size(0), cloud_vbo(QOpenGLBuffer::VertexBuffer),
    fbo     (c.getCameraWidth(), c.getCameraHeight(), false),
    fbo_msaa(c.getCameraWidth(), c.getCameraHeight(), true),
    hires_fbo(nullptr), hires_fbo_msaa(nullptr), hires_scale(0),
    ogl_log(this)
{
    connect(&ogl_log, &QOpenGLDebugLogger::messageLogged, this, &WorldRenderer::handleOpenGLmessage);

    if (camera.getNearClipping() == camera.getFarClipping()) {
        cerr << "ERROR: Invalid camera parameters supplied to WorldRenderer (near == far)." << endl;
        cerr << "       Make sure you defined near and far clipping parameters in your config." << endl;
        throw "Invalid camera parameters";
    }

    //setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
};

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::initializeGL()
{
    ogl_log.initialize();
    ogl_log.startLogging(QOpenGLDebugLogger::LoggingMode::SynchronousLogging); // use asynchronous logging for less overhead

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    f->glEnable(GL_DEPTH_TEST);
    f->glEnable(GL_PROGRAM_POINT_SIZE); // we want to render our points bigger if they are closer

    f->glViewport(0, 0, camera.getCameraWidth(), camera.getCameraHeight());

    // draw circles instead of squares
    /*
    // TODO: this isn't supported by OpenGL 3.0+ -> a custom shader is required :(
    f->glEnable(GL_POINT_SMOOTH);
    f->glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    */

    //T_GL_CV(0,0) = -1; // invert X axis
    T_GL_CV(1,1) = -1; // invert Y axis
    T_GL_CV(2,2) = -1; // invert Z axis

    m_gui_world.setToIdentity();
    m_gui_camera.setToIdentity();

    m_offscreen_proj.setToIdentity();
    m_offscreen_proj.perspective(
            qRadiansToDegrees(camera.getVerticalAngle()),
            GLfloat(camera.getCameraWidth())/camera.getCameraHeight(),
            camera.getNearClipping(), camera.getFarClipping());

    // not sure what Qt is exactly doing in perspective(), but this recreates the matrix from esim
    m_offscreen_proj(0,0) = 2 * camera.getFocalLength().x() / camera.getCameraSize().x();
    m_offscreen_proj(1,1) = 2 * camera.getFocalLength().y() / camera.getCameraSize().y();

    /*
    cout << "offscreen projection matrix:" << endl;
    qDebug() << m_offscreen_proj;

    cout << "camera size: " << camera.getCameraSize().transpose() << endl;
    cout << "camera angle vert: " << qRadiansToDegrees(camera.getVerticalAngle()) << endl;
    cout << "clipping: " << camera.getNearClipping() << " to " << camera.getFarClipping() << endl;
    */


    /*
     * esim projection matrix:
     *
       1.66667  0        0        0
       0  2.22222        0        0
       0        0 -1.05128 -2.05128
       0        0       -1        0

       fx: 200, fy: 200
       cx: 120, cy: 90
       width: 240, height: 180
       zmin: 1, zmax: 40

     */

    connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &WorldRenderer::cleanup);

    // set up framebuffer for off-screen rendering
    ////////////////////////////////////////////////////////////////////////////

    fbo.create();
    fbo_msaa.create();

    // load actual map (either a point cloud or a mesh)
    ////////////////////////////////////////////////////////////////////////////

    if (map_file.length() <= 4) {
        cerr << "ERROR: invalid map file" << endl;
    } else {

        switch (map_type) {
            case AUTO:
                if (map_file.substr(map_file.length()-4) == ".ply") {
                    if (!loadPointCloud(map_file)) {
                        cout << "WARNING: PLY contains faces, loading it as a mesh!" << endl;
                        loadMesh(map_file);
                    }
                } else if (map_file.substr(map_file.length()-4) == ".obj") {
                    loadMesh(map_file);
                } else {
                    cerr << "ERROR: invalid map file: File type '" << map_file.substr(map_file.length()-4) << "' not supported. Please provide either a '.ply' or a '.obj'." << endl;
                }
                break;

            case CLOUD:
                loadPointCloud(map_file);
                break;

            case AUTOSHADE_MESH:
            case SHADED_MESH:
            case UNSHADED_MESH:
                loadMesh(map_file);
                break;

            default:
                throw "Invalid map type. This should not happen at all. Something is corrupt!";
        }
    }

    emit ready();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::cleanup()
{
    makeCurrent();

    fbo.destroy();
    fbo_msaa.destroy();

    cloud_vbo.destroy();

    ogl_log.stopLogging();

    doneCurrent();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::handleOpenGLmessage(const QOpenGLDebugMessage& msg)
{
    if (msg.id() == 131185) {
        // omit messages about buffer memory allocation information
        return;
    }

    cout << "OpenGL (ID " << msg.id() << "): " << msg.message().toStdString() << endl;
}

////////////////////////////////////////////////////////////////////////////////

bool WorldRenderer::loadPointCloud(const std::string file)
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // load data from disk
    ////////////////////////////////////////////////////////////////////////////

    cout << "loading pointcloud from " << file << endl;

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file, 0);

    if (!scene) {
        throw "Loading point cloud failed. Does the file exist?";
    }

    if (scene->mNumMeshes > 1) {
        cerr << "WARNING: Point cloud contains more than one mesh. Only the first one will be loaded!" << endl;
    }

    aiMesh* mesh = scene->mMeshes[0];

    if (mesh->mNumFaces > 0) {
        cout << "number of faces: " << mesh->mNumFaces << endl;
        if (map_type == AUTO) {
            cout << " -> faces detected, treat this as a mesh. Use 'map_type: cloud' in your config to override this behaviour." << endl;
            return false; // this is actually a mesh!
        }
    }

    if (!mesh->HasNormals()) {
        throw "point cloud file does not contain normals";
    }

    vertices.resize(mesh->mNumVertices);
    for (size_t i = 0; i < mesh->mNumVertices; i++) {

        vertices[i].position = Eigen::Vector3f(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
        vertices[i].normal   = Eigen::Vector3f(mesh->mNormals[i].x,  mesh->mNormals[i].y,  mesh->mNormals[i].z);
        vertices[i].color    = Eigen::Matrix<uint8_t,3,1>(
                mesh->mColors[0][i].b*255, // we're following OpenCV here with BGR ordering... (not that it matters much, the actual tracking operates on grayscale data)
                mesh->mColors[0][i].g*255,
                mesh->mColors[0][i].r*255);
    }

    // set up shader for point cloud
    ////////////////////////////////////////////////////////////////////////////

    cloud_shader.addShaderFromSourceFile(QOpenGLShader::Vertex,   "opengl/shaders/cloud_color.vert");
    cloud_shader.addShaderFromSourceFile(QOpenGLShader::Fragment, "opengl/shaders/cloud_color.frag");

    cloud_shader.bindAttributeLocation("pos", 0);
    cloud_shader.bindAttributeLocation("normal", 1);
    cloud_shader.bindAttributeLocation("col", 2);

    cloud_shader.link();
    cloud_shader.bind();

    cloud_shader_u_MVP = cloud_shader.uniformLocation("MVP");
    cloud_shader_u_camera_pos = cloud_shader.uniformLocation("camera_pos");

    cloud_shader.release();

    // copy data to GPU
    ////////////////////////////////////////////////////////////////////////////

    cloud_vao.create();
    QOpenGLVertexArrayObject::Binder vao_binder(&cloud_vao);

    cloud_vbo.create();
    cloud_vbo.bind();
    cloud_vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    cloud_vbo.allocate(vertices.data(), vertices.size() * sizeof(CloudVertex));

    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CloudVertex), (void*)offsetof(CloudVertex, CloudVertex::position));
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CloudVertex), (void*)offsetof(CloudVertex, CloudVertex::normal));
    f->glEnableVertexAttribArray(2);
    f->glVertexAttribPointer(2, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(CloudVertex), (void*)offsetof(CloudVertex, CloudVertex::color));

    cloud_vbo.release();

    ////////////////////////////////////////////////////////////////////////////

    cloud_size = mesh->mNumVertices;
    //delete scene;

    cout << "loaded " << cloud_size << " points" << endl;

    return true;
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::loadMesh(const std::string file)
{
    cout << "loading mesh from " << file << endl;

    // load data from disk
    ////////////////////////////////////////////////////////////////////////////

    model.loadModel(file);

    // set up shader for mesh
    ////////////////////////////////////////////////////////////////////////////

    if (model.hasNormals() && (map_type == SHADED_MESH || map_type == AUTOSHADE_MESH)) {
        cout << "model has normals, enabling shading" << endl;
        mesh_shader.addShaderFromSourceFile(QOpenGLShader::Vertex,   "opengl/shaders/shaded_mesh.vert");
        mesh_shader.addShaderFromSourceFile(QOpenGLShader::Fragment, "opengl/shaders/shaded_mesh.frag");
    } else {
        if (map_type == SHADED_MESH) {
            cerr << "WARNING: model has no normals, disabling shading" << endl;
        }
        mesh_shader.addShaderFromSourceFile(QOpenGLShader::Vertex,   "opengl/shaders/mesh.vert");
        mesh_shader.addShaderFromSourceFile(QOpenGLShader::Fragment, "opengl/shaders/mesh.frag");
    }

    mesh_shader.bindAttributeLocation("aPos", 0);
    mesh_shader.bindAttributeLocation("aNormal", 1);
    mesh_shader.bindAttributeLocation("aTexCoords", 2);

    mesh_shader.link();
    mesh_shader.bind();

    mesh_shader_u_MVP = mesh_shader.uniformLocation("MVP");
    mesh_shader_u_M   = mesh_shader.uniformLocation("M");

    mesh_shader.release();
    cloud_size = 0;

    cout << "mesh loaded" << endl;
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::resizeGL(int w, int h)
{
    m_gui_proj.setToIdentity();
    m_gui_proj.perspective(qRadiansToDegrees(camera.getVerticalAngle()), GLfloat(w)/h, camera.getNearClipping(), camera.getFarClipping());

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, w, h);
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::paintGL()
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, width(), height());
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (cloud_size > 0) {
        paintCloud();
    } else {
        paintMesh();
    }
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::paintCloud()
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // activate vertex array and shaders
    QOpenGLVertexArrayObject::Binder vao_binder(&cloud_vao);
    cloud_shader.bind();

    // pass matrices to shader
    cloud_shader.setUniformValue(cloud_shader_u_MVP, m_gui_proj * m_gui_camera * m_gui_world);
    cloud_shader.setUniformValue(cloud_shader_u_camera_pos, widget_cam_T_WC.position.x(), widget_cam_T_WC.position.y(), widget_cam_T_WC.position.z());

    // actually draw our cloud
    f->glDrawArrays(GL_POINTS, 0, cloud_size);

    cloud_shader.release();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::paintMesh()
{
    // activate shaders
    mesh_shader.bind();

    // pass matrices to shader
    mesh_shader.setUniformValue(mesh_shader_u_MVP, m_gui_proj * m_gui_camera * m_gui_world);
    mesh_shader.setUniformValue(mesh_shader_u_M,   m_gui_world);

    // actually draw our mesh
    model.draw(mesh_shader);

    mesh_shader.release();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::set_gui_camera(Posef T_WC)
{
    widget_cam_T_WC = T_WC;

#if 1
    // convert OpenCV coordinate system into OpenGL one

    // we need T_CW for OpenGL, as we're converting world coordinates to camera coordinates
    m_gui_camera = T_GL_CV * T_WC.inverse().toQMatrix();
#else
    m_gui_camera = pos.inverse().toQMatrix();
    //m_gui_camera.rotate(90, 1, 0, 0); // rotate around X axis
    //m_gui_camera.rotate(90, 0, 1, 0); // rotate around Y axis
    //m_gui_camera.rotate(90, 0, 0, 1); // rotate around Z axis
#endif

    update(); // trigger a repaint
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::renderPose(const Posef& T_WC, cv::Mat& color, cv::Mat& depth, bool grayscale)
{
    makeCurrent();
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, camera.getCameraWidth(), camera.getCameraHeight());

    fbo_msaa.bind();


    // viewport is global (per context), so we have to change this here,
    // otherwise viewport from GUI widget is used
    //cout << "setting viewport to " << camera.getCameraWidth() << " x " << camera.getCameraHeight() << endl;
    f->glViewport(0, 0, camera.getCameraWidth(), camera.getCameraHeight());

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // TODO: write combined paint functions for offscreen and gui rendering

    if (!f->glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
        cerr << "ERROR: Failed to load framebuffer." << endl;
    }

    render(T_WC);

    fbo_msaa.blit_to_fbo(fbo);

    color = fbo.read_color(grayscale);
    depth = fbo.read_depth(camera);

    //cv::imshow("rendered by OpenGL", color);
    //cv::waitKey(1);

    fbo_msaa.release();

    doneCurrent();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::setupHiresRendering(int scale)
{
    makeCurrent();

    if (scale < 1) {
        scale = 1;
    }
    hires_scale = scale;

    hires_fbo      = new GLFramebuffer(camera.getCameraWidth()*scale, camera.getCameraHeight()*scale, false);
    hires_fbo_msaa = new GLFramebuffer(camera.getCameraWidth()*scale, camera.getCameraHeight()*scale, true);

    hires_fbo->create();
    hires_fbo_msaa->create();

    doneCurrent();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::cleanupHiresRendering()
{
    makeCurrent();

    delete hires_fbo;
    delete hires_fbo_msaa;

    doneCurrent();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::renderPoseHighres(const Posef& T_WC, cv::Mat& color, cv::Mat& depth)
{
    makeCurrent();
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glViewport(0, 0, camera.getCameraWidth()*hires_scale, camera.getCameraHeight()*hires_scale);


    hires_fbo_msaa->bind();


    // viewport is global (per context), so we have to change this here,
    // otherwise viewport from GUI widget is used
    //cout << "setting viewport to " << camera.getCameraWidth() << " x " << camera.getCameraHeight() << endl;
    f->glViewport(0, 0, camera.getCameraWidth()*hires_scale, camera.getCameraHeight()*hires_scale);

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // TODO: write combined paint functions for offscreen and gui rendering

    if (!f->glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
        cerr << "ERROR: Failed to load framebuffer." << endl;
    }

    render(T_WC);

    hires_fbo_msaa->blit_to_fbo(*hires_fbo);

    color = hires_fbo->read_color(false);
    depth = hires_fbo->read_depth(camera);

    hires_fbo_msaa->release();

    doneCurrent();
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::render(const Posef& T_WC)
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    QMatrix4x4 I;
    QMatrix4x4 T_CW = T_WC.inverse().toQMatrix();

    if (cloud_size > 0) {
        cout << "rendering cloud" << endl;

        // activate vertex array and shaders
        QOpenGLVertexArrayObject::Binder vao_binder(&cloud_vao);
        cloud_shader.bind();

        // pass matrices to shader
        cloud_shader.setUniformValue(cloud_shader_u_MVP,        m_offscreen_proj * T_GL_CV * T_CW);
        cloud_shader.setUniformValue(cloud_shader_u_camera_pos, T_WC.position.x(), T_WC.position.y(), T_WC.position.z());

        // actually draw our cloud
        f->glDrawArrays(GL_POINTS, 0, cloud_size);

        cloud_shader.release();

    } else {
        cout << "rendering mesh" << endl;
        // activate shaders
        mesh_shader.bind();

        // pass matrices to shader
        mesh_shader.setUniformValue(mesh_shader_u_MVP, m_offscreen_proj * T_GL_CV * T_CW);
        mesh_shader.setUniformValue(mesh_shader_u_M,   I);

        // actually draw our mesh
        model.draw(mesh_shader);

        mesh_shader.release();
    }
}

////////////////////////////////////////////////////////////////////////////////

Keyframe WorldRenderer::renderPose(const Posef& T_WC)
{
    cv::Mat intensity, depth;
    renderPose(T_WC, intensity, depth);
    Keyframe kf(intensity, depth, camera, ros::Time(1)); // time == 0 is interpreted as invalid KF
    kf.T_WK.pose = T_WC;
    kf.T_WK.valid = true;
    return kf;
}

////////////////////////////////////////////////////////////////////////////////

void WorldRenderer::publish(ros::Publisher& rospub_map)
{
    cout << "publishing our map to ROS" << endl;

    if (map_type != CLOUD) {
        cerr << "ERROR: Cannot publish mesh. Only clouds are currently supported" << endl;
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;

    cout << "We have " << vertices.size() << " points" << endl;

    cloud_msg.header.frame_id = "map";

    cloud_msg.width = vertices.size();
    cloud_msg.height = 1;

    cloud_msg.point_step = sizeof(CloudVertex);

    cloud_msg.is_dense = true;


#define ADD_FIELD(field_name, field_type, field_offset) do { \
        sensor_msgs::PointField pf; \
        pf.name = field_name; \
        pf.count = 1; \
        pf.datatype = field_type; \
        pf.offset = (field_offset); \
        cloud_msg.fields.push_back(pf); \
    } while(0)

    ADD_FIELD("x", sensor_msgs::PointField::FLOAT32, 0 * sizeof(float));
    ADD_FIELD("y", sensor_msgs::PointField::FLOAT32, 1 * sizeof(float));
    ADD_FIELD("z", sensor_msgs::PointField::FLOAT32, 2 * sizeof(float));
    ADD_FIELD("normal_x", sensor_msgs::PointField::FLOAT32, 3 * sizeof(float));
    ADD_FIELD("normal_y", sensor_msgs::PointField::FLOAT32, 4 * sizeof(float));
    ADD_FIELD("normal_z", sensor_msgs::PointField::FLOAT32, 5 * sizeof(float));
    ADD_FIELD("rgb", sensor_msgs::PointField::UINT32, 6 * sizeof(float) + 0);

    cloud_msg.data.resize(vertices.size()*sizeof(CloudVertex));
    memcpy(cloud_msg.data.data(), vertices.data(), vertices.size()*sizeof(CloudVertex));


    rospub_map.publish(cloud_msg);
}

////////////////////////////////////////////////////////////////////////////////

