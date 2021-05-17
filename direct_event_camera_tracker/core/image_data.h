#ifndef IMAGE_DATA_H_INCLUDED
#define IMAGE_DATA_H_INCLUDED

#include <cmath>
#include <string>
#include <iostream>
#include <limits>
#include <Eigen/Dense>

/*!
 * This class represents a basic image, with some additional helper functions
 *
 */
class ImageData
{
public:

    //! allocates memory, but doesn't initialize it!
    void create(const unsigned int W, const unsigned int H);

    //! load data
    void loadFromMatrix(const Eigen::MatrixXf& source_data);

    //! scale values, so that 0 -> near, 1 -> far
    void scaleTo(double near, double far);

    inline unsigned int getWidth()  const { return data.cols(); }
    inline unsigned int getHeight() const { return data.rows(); }
    inline Eigen::Vector2d getSize()   const { return Eigen::Vector2d(getWidth(), getHeight()); }

    inline const Eigen::MatrixXd& getData() const { return data; }
    inline       Eigen::MatrixXd& getData()       { return data; }
    inline double  getValue(Eigen::Vector2i pos) const { return data(pos.y(), pos.x()); }
    inline double& getValue(Eigen::Vector2i pos)       { return data(pos.y(), pos.x()); }
    inline double& operator()(unsigned int x, unsigned int y) { return data(y,x); }

    double sampleValue(Eigen::Vector2d pos) const;
    Eigen::Matrix<double, 1, 2> sampleDiff(Eigen::Vector2d pos) const;
    Eigen::Matrix<double, 1, 2> getDiff(Eigen::Vector2i pos) const;

    // halve image size
    void downsample2();



    // authorative data
    Eigen::MatrixXd data;
};

#endif /* end of include guard: IMAGE_DATA_H_INCLUDED */
