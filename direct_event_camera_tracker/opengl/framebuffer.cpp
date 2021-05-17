#include "framebuffer.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

void GLFramebuffer::create()
{
    QOpenGLContext* ctx = QOpenGLContext::currentContext();

    if (!ctx) {
        throw "Cannot initialize framebuffer: No current OpenGL context.";
    }

    QOpenGLFunctions_3_3_Core* f = ctx->versionFunctions<QOpenGLFunctions_3_3_Core>();

    if (!f) {
        // this should really not happen (unless your GPU is way too old)
        throw "Cannot initialize framebuffer: Cannot get OpenGL 3.3 features.";
    }

    // generate framebuffer object

    // create framebuffer object
    f->glGenFramebuffers(1, &fbo);
    f->glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // query number of supported samples
    int samples = 8;
    if (multisampled) {
        GLint supported_samples = 0;
        GLint supported_int_samples = 0;
        f->glGetIntegerv(GL_MAX_SAMPLES, &supported_samples);
        f->glGetIntegerv(GL_MAX_INTEGER_SAMPLES, &supported_int_samples);

        if (supported_samples <= 1 or supported_int_samples <= 1)
        {
            cerr << "WARNING: Multisampling (anti-aliasing) not supported by your graphics card drivers: Disabling multisampling" << endl;
            multisampled = false;
        }
        else if (supported_samples < samples or supported_int_samples < samples)
        {
            samples = min(supported_samples, supported_int_samples);
            cout << "INFO: Multisampling (anti-aliasing) only supported up to " << samples << " samples." << endl;
        }
    }

    // create and attach a color buffer
    f->glGenRenderbuffers(1, &color_buf);
    f->glBindRenderbuffer(GL_RENDERBUFFER, color_buf);
    if (multisampled) {
        f->glRenderbufferStorageMultisample(GL_RENDERBUFFER, 8, GL_RGBA8, width, height); // set format
    } else {
        f->glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height); // set format
    }
    f->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_buf); // attach color buffer to FBO

    // create and attach a depth buffer
    f->glGenRenderbuffers(1, &depth_buf);
    f->glBindRenderbuffer(GL_RENDERBUFFER, depth_buf);
    if (multisampled) {
        f->glRenderbufferStorageMultisample(GL_RENDERBUFFER, 8, GL_DEPTH_COMPONENT24, width, height); // set format
    } else {
        f->glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height); // set format
    }
    f->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_buf); // attach depth buffer to FBO

    if (!f->glCheckFramebufferStatus(GL_FRAMEBUFFER)) {
        cerr << "ERROR: Failed to set up framebuffer." << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

void GLFramebuffer::destroy()
{
    if (is_initialized) {
        QOpenGLContext* ctx = QOpenGLContext::currentContext();

        if (!ctx) {
            throw "Cannot clean up framebuffer: No current OpenGL context.";
        }

        QOpenGLFunctions* f = ctx->functions();

        f->glDeleteRenderbuffers(1, &color_buf);
        f->glDeleteRenderbuffers(1, &depth_buf);
        f->glDeleteFramebuffers(1, &fbo);

        is_initialized = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////

void GLFramebuffer::bind()
{
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glBindFramebuffer(GL_FRAMEBUFFER, fbo);
}

////////////////////////////////////////////////////////////////////////////////

void GLFramebuffer::release()
{
    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    ctx->functions()->glBindFramebuffer(GL_FRAMEBUFFER, ctx->defaultFramebufferObject());
}

////////////////////////////////////////////////////////////////////////////////

void GLFramebuffer::blit_to_fbo(GLFramebuffer& target)
{
    QOpenGLFunctions_3_3_Core* f = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();

    f->glBindFramebuffer(GL_READ_FRAMEBUFFER, this->fbo);
    f->glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target.fbo);

    f->glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat GLFramebuffer::read_color(bool grayscale)
{
    if (multisampled) {
        cerr << "ERROR: Cannot read color from multisampled framebuffer. Blit it to a normal FBO first!" << endl;
        return cv::Mat();
    }

    bind();

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // allocate some space
    cv::Mat img(height, width, CV_8UC3);

    f->glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
    f->glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());

    f->glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data);

    // image coordinates are vertically inverted between OpenCV and OpenGL
    cv::flip(img, img, 0);

    // convert color image to grayscale
    if (grayscale) {
        cv::Mat img_grayscale;
        cv::cvtColor(img, img_grayscale, cv::COLOR_BGR2GRAY);

        // convert to double
        img_grayscale.convertTo(img, CV_64FC1, 1/255.0f);
    } else {
        img.convertTo(img, CV_64FC3, 1/255.0f);
    }

    release();

    return img;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat GLFramebuffer::read_depth(const CameraIntrinsics& camera)
{
    if (multisampled) {
        cerr << "ERROR: Cannot read depth from multisampled framebuffer. Blit it to a normal FBO first!" << endl;
        return cv::Mat();
    }

    bind();

    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();

    // allocate some space
    cv::Mat img(height, width, CV_32FC1);

    f->glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
    f->glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize());

    f->glReadPixels(0, 0, img.cols, img.rows, GL_DEPTH_COMPONENT, GL_FLOAT, img.data);

    double min_d, max_d;
    cv::minMaxLoc(img, &min_d, &max_d);
    //cout << "raw depth from " << min_d << " to " << max_d << endl;

    if (min_d == max_d && min_d != 1) { // all 1 is possible if there was nothing in the scene (which shouldn't but might happen)
        cerr << "ERROR: Failed to read depth from framebuffer." << endl;
        return cv::Mat();
    }

    // image coordinates are vertically inverted between OpenCV and OpenGL
    cv::flip(img, img, 0);

    // convert from floats to doubles
    img.convertTo(img, CV_64FC1); // TODO: we could possibly do the conversion below in this step too

    // convert inverse depth buffer to linear depth between zmin and zmax
    // see the "Learn OpenGL book, page 177
    cv::Mat linear_depth = (2.0 * camera.getNearClipping() * camera.getFarClipping())
        / (camera.getNearClipping() + camera.getFarClipping() - (2 * img - 1.f) * (camera.getFarClipping() - camera.getNearClipping()));

    //cv::minMaxLoc(linear_depth, &min_d, &max_d);
    //cout << "linearized depth from " << min_d << " to " << max_d << endl;

    release();

    return linear_depth;
}

////////////////////////////////////////////////////////////////////////////////

