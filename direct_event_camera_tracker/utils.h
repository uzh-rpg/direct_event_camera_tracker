#ifndef UTILS_H_YFZCGEVA
#define UTILS_H_YFZCGEVA

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <functional>
#include <limits>
#include <cxxabi.h>

#include <QSizeF>
#include <QPointF>
#include <QVector>

#include "polyfit.h"

////////////////////////////////////////////////////////////////////////////////

#define UNUSED(x) ((void)(x))

////////////////////////////////////////////////////////////////////////////////

void image_gradient    (const cv::Mat& src, cv::Mat& dst);
void gradient_of_gradient(const cv::Mat& src, cv::Mat& dst);
void visualize_gradient(const cv::Mat& src, cv::Mat& dst);

cv::Mat visualize_correlation(const cv::Mat& src);

cv::Mat plot_gradient_legend();

////////////////////////////////////////////////////////////////////////////////

template <typename T>
T bilinear_interp(const cv::Mat& src, const Eigen::Vector2d& pos)
{
    Eigen::Vector2i p = pos.array().floor().cast<int>();

    // remember: data[row, col] = data[y, x]!
    const T v0 = src.at<T>(p.y(),   p.x());
    const T v1 = src.at<T>(p.y(),   p.x()+1);
    const T v2 = src.at<T>(p.y()+1, p.x());
    const T v3 = src.at<T>(p.y()+1, p.x()+1);

    // weight (how close to img[ceil(x), ceil(y)] are we?
    Eigen::Vector2d s = pos - p.cast<double>();

    return v0 * ( (1-s.x()) * (1-s.y()) )
         + v1 * ( (  s.x()) * (1-s.y()) )
         + v2 * ( (1-s.x()) * (  s.y()) )
         + v3 * ( (  s.x()) * (  s.y()) );
}

////////////////////////////////////////////////////////////////////////////////

template <typename MatT, typename T>
void draw_bilinear(cv::Mat& dst, const Eigen::Vector2d& pos, const T& val)
{
    Eigen::Vector2i p = pos.array().floor().cast<int>();

    // weight (how close to img[ceil(x), ceil(y)] are we?
    Eigen::Vector2d s = pos - p.cast<double>();


    if (p.x() < 0 || p.y() < 0 || p.x()+1 >= dst.cols || p.y()+1 >= dst.rows) {
        std::cerr << "ERROR: drawing bilinear at " << p.x() << ", " << p.y() << " (+ " << s.x() << ", " << s.y() << ")" << std::endl;
        std::cerr << "       But dst is " << dst.size() << std::endl;
    }

    // remember: data[row, col] = data[y, x]!
    dst.at<MatT>(p.y(),   p.x())       += val * ((1-s.x()) * (1-s.y()));
    dst.at<MatT>(p.y(),   p.x()+1)     += val * ((  s.x()) * (1-s.y()));
    dst.at<MatT>(p.y()+1, p.x())       += val * ((1-s.x()) * (  s.y()));
    dst.at<MatT>(p.y()+1, p.x()+1)     += val * ((  s.x()) * (  s.y()));
}

std::ostream & operator<<(std::ostream &out, const QSizeF& size);
std::ostream & operator<<(std::ostream &out, const QSize& size);
std::ostream & operator<<(std::ostream &out, const QPointF& point);
std::ostream & operator<<(std::ostream &out, const QPoint& point);

void downsample2_img(const cv::Mat& src, cv::Mat& dst);


// determine min(norm(p)) and max(norm(p))
void minmax_sqnorm(const cv::Mat& src, double& out_min, double& out_max, const cv::Mat& mask);

// count the number of pixels in src with norm(p)^2 >= thresh
size_t count_above(const cv::Mat& src, const double& thresh, const cv::Mat& mask);

// find and return approximate x such that count(norm(p)^2 >= x: p in src) == fraction*count(src) == out_count
double find_threshold(const cv::Mat& src, const double& fraction, size_t& out_count, const cv::Mat& mask);

////////////////////////////////////////////////////////////////////////////////

template <class T>
typename std::vector<T>::const_iterator find_closest(
        const typename std::vector<T>& vec,
        const std::function<double(const T&)> eval)
{
    return find_closest(vec.begin(), vec.end(), eval);
}

////////////////////////////////////////////////////////////////////////////////
// search element in sorted vector that is closest to 0
// i.e. return argmin_T abs(eval(T))
//
// eval(T) -> double:
//     return values > 0 if T is above and < 0 if it is below the value you're
//     looking for
//
// this assumes eval(*begin) < eval(*end) !

template <class T>
typename std::vector<T>::const_iterator find_closest(
        const typename std::vector<T>::const_iterator& begin,
        const typename std::vector<T>::const_iterator& end,
        const std::function<double(const T&)> eval)
{
    if (begin == end) {
        throw "Cannot find closest element in vector: Vector is empty!";
    }

    // go to last element
    typename std::vector<T>::const_iterator last = end-1;

    if (begin == last) {
        return begin;
    }

    double left_val  = eval(*begin);

    // is needle before our range?
    if (left_val >= 0) {
        return begin;
    }

    double right_val = eval(*last);

    // is it after our range?
    if (right_val <= 0) {
        return last;
    }

    // needle is somewhere between begin and end
    // -> guess closest entry

    int dist = distance(begin, last);

    if (dist == 1) {
        // take the closer one
        if (std::abs(left_val) < std::abs(right_val)) {
            return begin;
        } else  {
            return last;
        }
    }

    int pivot_idx = round(left_val * dist / (left_val - right_val));

    assert (pivot_idx >= 0);
    assert (pivot_idx <= dist);

    if (pivot_idx == 0) {
        pivot_idx++;
    } else if (pivot_idx == dist) {
        pivot_idx--;
    }

    typename std::vector<T>::const_iterator pivot = begin + pivot_idx;

    double pivot_val = eval(*pivot);

    if (pivot_val == 0) {
        // found an exact match!
        return pivot;
    } else if (pivot_val >= 0) {
        // we guessed too high
        return find_closest(begin, pivot+1, eval); // do we need this +1 here?
    } else {
        // guessed too low
        return find_closest(pivot, end, eval);
    }
}

////////////////////////////////////////////////////////////////////////////////

template <class T>
Eigen::Vector2i toVec2i(const T& obj)
{
    return Eigen::Vector2i(obj.x, obj.y);
}

Eigen::Vector2i toVec2i(const cv::Size& obj);

////////////////////////////////////////////////////////////////////////////////

// return human-readable name of a type
// source: https://stackoverflow.com/a/19123821
template<typename T>
std::string type_name()
{
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name = abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if(status == 0) {
        tname = demangled_name;
        std::free(demangled_name);
    }
    return tname;
}

////////////////////////////////////////////////////////////////////////////////

std::string time_to_str(const ros::Time& t);

////////////////////////////////////////////////////////////////////////////////

QVector<double> moving_average(const QVector<double>& data, const int window_size);

////////////////////////////////////////////////////////////////////////////////

/*
 * macro to quickly define structs
 *
 * automatically generates constructor and declares variables
 *
 * use like this:

struct Foo {
    AUTO_STRUCT(
        Foo,
        ATTR(cv::Mat&, bla),
        ATTR(bool, blabla, false),
    )
};

 */


// helper macro to be able to "overload" a macro based on number of parameters
// ATTR(t,n)   -> ATTR_NODEF(t,n)
// ATTR(t,n,d) -> ATTR_DEF(t,n,d)
#define __NARG_3(_1, _2, _3, NAME, ...) NAME
#define __NARG_20(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, NAME, ...) NAME

// see eg https://stackoverflow.com/questions/1872220/is-it-possible-to-iterate-over-arguments-in-variadic-macros

// without comma separator
#define  FE1(f, x)      f(x)
#define  FE2(f, x, ...) f(x)  FE1(f, __VA_ARGS__)
#define  FE3(f, x, ...) f(x)  FE2(f, __VA_ARGS__)
#define  FE4(f, x, ...) f(x)  FE3(f, __VA_ARGS__)
#define  FE5(f, x, ...) f(x)  FE4(f, __VA_ARGS__)
#define  FE6(f, x, ...) f(x)  FE5(f, __VA_ARGS__)
#define  FE7(f, x, ...) f(x)  FE6(f, __VA_ARGS__)
#define  FE8(f, x, ...) f(x)  FE7(f, __VA_ARGS__)
#define  FE9(f, x, ...) f(x)  FE8(f, __VA_ARGS__)
#define FE10(f, x, ...) f(x)  FE9(f, __VA_ARGS__)
#define FE11(f, x, ...) f(x) FE10(f, __VA_ARGS__)
#define FE12(f, x, ...) f(x) FE11(f, __VA_ARGS__)
#define FE13(f, x, ...) f(x) FE12(f, __VA_ARGS__)
#define FE14(f, x, ...) f(x) FE13(f, __VA_ARGS__)
#define FE15(f, x, ...) f(x) FE14(f, __VA_ARGS__)
#define FE16(f, x, ...) f(x) FE15(f, __VA_ARGS__)
#define FE17(f, x, ...) f(x) FE16(f, __VA_ARGS__)
#define FE18(f, x, ...) f(x) FE17(f, __VA_ARGS__)
#define FE19(f, x, ...) f(x) FE18(f, __VA_ARGS__)
#define FE20(f, x, ...) f(x) FE19(f, __VA_ARGS__)
#define FE21(f, x, ...) f(x) FE20(f, __VA_ARGS__)

#define FOREACH(f, ...) __NARG_20(__VA_ARGS__, FE20, FE19, FE18, FE17, FE16, FE15, FE14, FE13, FE12, FE11, FE10, FE9, FE8, FE7, FE6, FE5, FE4, FE3, FE2, FE1)(f,__VA_ARGS__)


// with comma separator
#define  FE1C(f, x)      f(x)
#define  FE2C(f, x, ...) f(x) ,  FE1C(f, __VA_ARGS__)
#define  FE3C(f, x, ...) f(x) ,  FE2C(f, __VA_ARGS__)
#define  FE4C(f, x, ...) f(x) ,  FE3C(f, __VA_ARGS__)
#define  FE5C(f, x, ...) f(x) ,  FE4C(f, __VA_ARGS__)
#define  FE6C(f, x, ...) f(x) ,  FE5C(f, __VA_ARGS__)
#define  FE7C(f, x, ...) f(x) ,  FE6C(f, __VA_ARGS__)
#define  FE8C(f, x, ...) f(x) ,  FE7C(f, __VA_ARGS__)
#define  FE9C(f, x, ...) f(x) ,  FE8C(f, __VA_ARGS__)
#define FE10C(f, x, ...) f(x) ,  FE9C(f, __VA_ARGS__)
#define FE11C(f, x, ...) f(x) , FE10C(f, __VA_ARGS__)
#define FE12C(f, x, ...) f(x) , FE11C(f, __VA_ARGS__)
#define FE13C(f, x, ...) f(x) , FE12C(f, __VA_ARGS__)
#define FE14C(f, x, ...) f(x) , FE13C(f, __VA_ARGS__)
#define FE15C(f, x, ...) f(x) , FE14C(f, __VA_ARGS__)
#define FE16C(f, x, ...) f(x) , FE15C(f, __VA_ARGS__)
#define FE17C(f, x, ...) f(x) , FE16C(f, __VA_ARGS__)
#define FE18C(f, x, ...) f(x) , FE17C(f, __VA_ARGS__)
#define FE19C(f, x, ...) f(x) , FE18C(f, __VA_ARGS__)
#define FE20C(f, x, ...) f(x) , FE19C(f, __VA_ARGS__)
#define FE21C(f, x, ...) f(x) , FE20C(f, __VA_ARGS__)

#define FOREACHC(f, ...) __NARG_20(__VA_ARGS__, FE20C, FE19C, FE18C, FE17C, FE16C, FE15C, FE14C, FE13C, FE12C, FE11C, FE10C, FE9C, FE8C, FE7C, FE6C, FE5C, FE4C, FE3C, FE2C, FE1)(f,__VA_ARGS__)

#define ATTR(...) __NARG_3(__VA_ARGS__, ATTR_DEF, ATTR_NODEF)(__VA_ARGS__)

#define AS_INIT(a) AS_INIT_(a)
#define AS_INIT_(a) AS_INIT_ ## a
#define AS_INIT_ATTR_DEF(type, name, default_val)     name(name)
#define AS_INIT_ATTR_NODEF(type, name)                name(name)

#define AS_CONSTR(a) AS_CONSTR_(a)
#define AS_CONSTR_(a) AS_CONSTR_ ## a
#define AS_CONSTR_ATTR_DEF(type, name, default_val)   type name = default_val
#define AS_CONSTR_ATTR_NODEF(type, name)              type name

#define AS_DECLARE(a) AS_DECLARE_(a)
#define AS_DECLARE_(a) AS_DECLARE_ ## a
#define AS_DECLARE_ATTR_DEF(type, name, default_val)   type name;
#define AS_DECLARE_ATTR_NODEF(type, name)              type name;

#define AUTO_STRUCT(name, ...) \
    name(FOREACHC(AS_CONSTR, __VA_ARGS__)) \
    : FOREACHC(AS_INIT, __VA_ARGS__) {}; \
    \
    FOREACH(AS_DECLARE, __VA_ARGS__)


////////////////////////////////////////////////////////////////////////////////


#endif /* end of include guard: UTILS_H_YFZCGEVA */
