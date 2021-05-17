#include "camera_image.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
void CameraImage::loadFromMatrices(const Eigen::MatrixXd& mat_intensities, const Eigen::MatrixXd& mat_depths)
{
    intensities.loadFromMatrix(mat_intensities);
    depths     .loadFromMatrix(mat_depths);
}
///////////////////////////////////////////////////////////////////////////////
Pixel CameraImage::getPixel(Eigen::Vector2i pos) const
{
    assert(pos.x() >= 0);
    assert(pos.y() >= 0);
    assert(pos.x() < this->getWidth());
    assert(pos.y() < this->getHeight());

    Pixel p;
    p.pos(0) = pos.x();
    p.pos(1) = pos.y();
    p.intensity = intensities.getValue(pos);
    p.depth     = depths.getValue(pos);

    return p;
}
///////////////////////////////////////////////////////////////////////////////
double CameraImage::samplePixel(Eigen::Vector2f pos) const
{
    assert(isValidPixel(pos));
    return intensities.sampleValue(pos);
}
///////////////////////////////////////////////////////////////////////////////
bool CameraImage::isValidPixel(Eigen::Vector2f pos) const
{
    // center of top-left pixel is at [0,0]
    // center of bottom-right pixel is at [W-1, H-1]
    // therefore, both these coordinates are valid
    return pos.x() >= 0 && floor(pos.x()) <= getWidth()  - 2
        && pos.y() >= 0 && floor(pos.y()) <= getHeight() - 2;
}
///////////////////////////////////////////////////////////////////////////////
