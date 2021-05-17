#include "image_data.h"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
void ImageData::create(const unsigned int W, const unsigned int H)
{
    assert(W > 0); assert(H > 0);

    data.resize(H,W);
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::loadFromMatrix(const Eigen::MatrixXf& source_data)
{
    // copy data
    data = source_data;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::scaleTo(double near, double far)
{
    data = data.array()*(far-near) + near;
}
///////////////////////////////////////////////////////////////////////////////
double ImageData::sampleValue(Eigen::Vector2f pos) const
{
    assert(pos.x() >= 0);
    assert(pos.y() >= 0);
    assert(pos.x() <= getWidth()-1);
    assert(pos.y() <= getHeight()-1);

    // remember: data[row, col] = data[y, x]!
    double v1 = data(floor(pos.y()), floor(pos.x()));
    double v2 = data(floor(pos.y()),  ceil(pos.x()));
    double v3 = data( ceil(pos.y()), floor(pos.x()));
    double v4 = data( ceil(pos.y()),  ceil(pos.x()));

    // weight (how close to img[ceil(x), ceil(y)] are we?
    double sx = pos.x() - floor(pos.x());
    double sy = pos.y() - floor(pos.y());

    assert(sx >= 0); assert(sx < 1);
    assert(sy >= 0); assert(sy < 1);

    return v1 * ( (1-sx) * (1-sy) )
         + v2 * (    sx  * (1-sy) )
         + v3 * ( (1-sx) *    sy  )
         + v4 * (    sx  *    sy  );
}
///////////////////////////////////////////////////////////////////////////////
// WARNING: This function has a very slight bias, as points with integer
// coordinates (i.e. falling exactly on a pixel center) are not handled
// specially. Instead, the gradient towards the next (right/lower) pixel is
// taken.
Eigen::Matrix<double, 1, 2> ImageData::sampleDiff(Eigen::Vector2d pos) const
{
    double fx = floor(pos.x()), fy = floor(pos.y());

    if (fx > getWidth()-2) {
        cout << "fail: " << pos.transpose() << " --> " << fx << " " << fy << endl;
        cout << "W: " << getWidth() << " H: " << getHeight() << endl;
    }

    // make sure, all four neighbouring points lie inside the image
    assert(fx >= 0);
    assert(fy >= 0);
    assert(fx <= getWidth()-2);
    assert(fy <= getHeight()-2);

    // remember: data[row, col] = data[y, x]!
    double v1 = data(fy,   fx);
    double v2 = data(fy,   fx+1);
    double v3 = data(fy+1, fx);
    double v4 = data(fy+1, fx+1);

    // weight (how close to img[ceil(x), ceil(y)] are we?
    double sx = pos.x() - fx;
    double sy = pos.y() - fy;

    double dx1 = v2 - v1,  dy1 = v3 - v1;
    double dx2 = v4 - v3,  dy2 = v4 - v2;

    Eigen::Matrix<double, 1, 2> J;

    J(0) = dx1 * (1-sy)  +  dx2 * sy;
    J(1) = dy1 * (1-sx)  +  dy2 * sx;

    return J;
}
///////////////////////////////////////////////////////////////////////////////
// samples the image gradient at the exact pixel center, using a -1,0,1 kernel (or -1 1 on the edge)
Eigen::Matrix<double, 1, 2> ImageData::getDiff(Eigen::Vector2i pos) const
{
    double diffx = 0, diffy = 0;

    if (pos.x() <= 0) {
        // no pixel on the left, just use gradient from this to the one on the right
        diffx = getValue(pos+Eigen::Vector2i(1,0)) - getValue(pos);
    } else if (pos.x() >= (double) getWidth()-1) {
        // no pixel on the right, just use gradient from the one on the left to this pixel
        diffx = getValue(pos) - getValue(pos+Eigen::Vector2i(-1,0));
    } else {
        diffx = (getValue(pos + Eigen::Vector2i( 1,0))
                -getValue(pos + Eigen::Vector2i(-1,0))) / 2;
    }

    if (pos.y() <= 0) {
        diffy = getValue(pos+Eigen::Vector2i(0,1)) - getValue(pos);
    } else if (pos.y() >= (double) getHeight()-1) {
        diffy = getValue(pos) - getValue(pos+Eigen::Vector2i(0,-1));
    } else {
        diffy = (getValue(pos + Eigen::Vector2i(0,1))
                -getValue(pos + Eigen::Vector2i(0,-1))) / 2;
    }

    Eigen::Matrix<double, 1, 2> J;

    J(0) = diffx;
    J(1) = diffy;

    return J;
}
///////////////////////////////////////////////////////////////////////////////
void ImageData::downsample2()
{
    for (int r = 0; r < data.rows()/2; r++) {
        for (int c = 0; c < data.cols()/2; c++) {
            data(r,c) = (data(2*r,   2*c)
                      +  data(2*r+1, 2*c)
                      +  data(2*r,   2*c+1)
                      +  data(2*r+1, 2*c+1)) / 4;
        }
    }

    data.conservativeResize(getHeight()/2, getWidth()/2);
}
///////////////////////////////////////////////////////////////////////////////
