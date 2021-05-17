#ifndef SCENE_IMAGE_H_INCLUDED
#define SCENE_IMAGE_H_INCLUDED

#include <vector>
#include <Eigen/Dense>

#include "image_data.h"

///////////////////////////////////////////////////////////////////////////////

struct Pixel
{
    Pixel(double x = 0, double y = 0, double depth = 0, double intensity = 0)
        : pos(x,y), intensity(intensity), depth(depth) {}

    Eigen::Vector2d pos;
    double intensity;
    double depth;
};

///////////////////////////////////////////////////////////////////////////////

struct WorldPoint
{
    WorldPoint(double x = 0, double y = 0, double z = 0, Pixel p = Pixel())
        : pos(x,y,z), pixel(p) {}

    Eigen::Vector3d pos;
    Pixel pixel;
};

///////////////////////////////////////////////////////////////////////////////

/*!
 * an image with its depth map
 * i.e. a single datapoint from the stereo camera
 */
struct CameraImage
{
    void loadFromMatrices(const Eigen::MatrixXd& mat_intensities, const Eigen::MatrixXd& mat_depths);

    inline int getWidth()  const { return intensities.getWidth(); }
    inline int getHeight() const { return intensities.getHeight(); }

    Pixel getPixel   (Eigen::Vector2i pos) const; // directly read out pixel from integer coordinates
    double samplePixel(Eigen::Vector2d pos) const; // linearly interpolate between four pixels from double coordinates

    bool isValidPixel(Eigen::Vector2d pos) const;

    inline const ImageData& getIntensityData() const { return intensities; }
    inline const ImageData& getDepthData()     const { return depths; }

    void downsample2() { intensities.downsample2(); depths.downsample2(); }

    ImageData intensities, depths;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: SCENE_IMAGE_H_INCLUDED */
