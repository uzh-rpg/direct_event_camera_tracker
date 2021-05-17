#ifndef FRAMEBUFFER_H_R3LDU7KT
#define FRAMEBUFFER_H_R3LDU7KT

#include <iostream>

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_3_3_Core>

#include <opencv2/opencv.hpp>

#include "core/camera_intrinsics.h"

class GLFramebuffer
{
public:
    GLFramebuffer(unsigned int width, unsigned int height, bool multisampled=false)
        : is_initialized(false), fbo(0), color_buf(0), depth_buf(0),
          width(width), height(height), multisampled(multisampled)
    {
    }

    ~GLFramebuffer() { destroy(); }

    void create();
    void destroy();

    void bind();
    void release();

    void blit_to_fbo(GLFramebuffer& target);

    cv::Mat read_color(bool grayscale = true);
    cv::Mat read_depth(const CameraIntrinsics& camera);

private:
    bool is_initialized;

    GLuint fbo;
    GLuint color_buf, depth_buf;

    unsigned int width, height;
    bool multisampled;
};

#endif /* end of include guard: FRAMEBUFFER_H_R3LDU7KT */
