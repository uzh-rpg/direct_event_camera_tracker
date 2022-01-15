#ifndef WORLD_RENDERER_H_JHO7KVQG
#define WORLD_RENDERER_H_JHO7KVQG

#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_0>
#include <QOpenGLFunctions_3_3_Core>
#include <QMatrix4x4>
#include <QMatrix3x3>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QtMath>
#include <QOpenGLDebugLogger>
#include <QSizePolicy>

#include <Eigen/Core>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "core/camera_intrinsics.h"
#include "opengl/lib/model.h"
#include "opengl/framebuffer.h"
#include "point.h"
#include "keyframe.h"

enum MapType {
    AUTO,
    AUTOSHADE_MESH,
    SHADED_MESH,
    UNSHADED_MESH,
    CLOUD
};

std::ostream& operator<<(std::ostream& os, const MapType& map_type);

struct CloudVertex
{
    Eigen::Vector3f position,
                    normal;
    Eigen::Matrix<uint8_t, 3, 1>
                    color;
};

class WorldRenderer : public QOpenGLWidget
{
    Q_OBJECT
public:
    WorldRenderer(QWidget* parent, const CameraIntrinsics& c);
    ~WorldRenderer() { cleanup(); }

    void loadMap(const std::string file, MapType type = MapType::AUTO) { map_file = file; map_type = type; }

    Keyframe renderPose(const Posef& T_WC);
    void renderPose(const Posef& T_WC, cv::Mat& color, cv::Mat& depth, bool grayscale = true);

    void setupHiresRendering(int scale);
    void renderPoseHighres(const Posef& T_WC, cv::Mat& color, cv::Mat& depth);
    void cleanupHiresRendering();

    virtual const QSize sizeHint() { return QSize(camera.getCameraWidth(), camera.getCameraHeight()); }

    void publish(ros::Publisher& rospub_map);


signals:
    void ready(); // emitted once OpenGL has been initialized and WorldRenderer can be used to render views

public slots:
    void set_gui_camera(Posef T_WC);

protected:
    // called by Qt
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();


    void paintCloud();
    void paintMesh();

    // called in initializeGL():
    bool loadPointCloud(const std::string file);
    void loadMesh(const std::string file);

protected slots:
    void cleanup();
    void handleOpenGLmessage(const QOpenGLDebugMessage& msg);

private:
    const CameraIntrinsics& camera;

    void render(const Posef& T_WC);

    // transformation matrices for visible widget
    QMatrix4x4 m_gui_proj;
    QMatrix4x4 m_gui_camera; // camera position and orientation
    QMatrix4x4 m_gui_world;  // world pos & orientation (TODO: this is actually not required as model is always at origin)
    Posef widget_cam_T_WC;

    // path to world map
    std::string map_file;
    MapType map_type;

    // things related to point cloud
    QOpenGLShaderProgram cloud_shader;
    int cloud_shader_u_MVP, cloud_shader_u_camera_pos;
    std::vector<CloudVertex> vertices;

    size_t cloud_size;

    //pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_data;
    QOpenGLVertexArrayObject cloud_vao; // vertex array
    QOpenGLBuffer cloud_vbo; // vertex buffer

    // things related to mesh
    Model model;
    QOpenGLShaderProgram mesh_shader;
    int mesh_shader_u_MVP, mesh_shader_u_M;

    // for off-screen rendering
    GLFramebuffer fbo, fbo_msaa;
    GLFramebuffer *hires_fbo, *hires_fbo_msaa; int hires_scale;
    QMatrix4x4 m_offscreen_proj;

    // general OpenGL things
    QOpenGLDebugLogger ogl_log;
    QMatrix4x4 T_GL_CV; // conversion from OpenCV coordinate system to OpenGL one
};

#endif /* end of include guard: WORLD_RENDERER_H_JHO7KVQG */
