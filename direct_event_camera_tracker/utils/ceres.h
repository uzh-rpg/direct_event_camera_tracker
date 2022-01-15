#ifndef CERES_H_DPURA6XC
#define CERES_H_DPURA6XC

#include <Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>

#include <boost/function.hpp>

#include <type_traits>

#include "point.h"
#include "utils.h"


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// helper code to cast ceres::Jet's to int/float/double
// this is used to assign data to images in the error<double> function
// alternatively, some kind of static_if could be used to ignore the output
// images during error calculations.
// It can also be used to output Scalars easily

//#define INFO do { std::cout << "[" << type_name<_from_t>() <<  " -> " << type_name<_to_t>() << "]"; } while(0)
#define INFO

namespace CeresCaster
{
    // Jet

    template <typename T, int N>
    double toDouble(const ceres::Jet<T,N>& val) { return val.a; }

    template <typename T, int N>
    float toFloat(const ceres::Jet<T,N>& val) { return val.a; }


    template <int R, int C, typename T, int N>
    Eigen::Matrix<double,R,C> toDoubleV(const Eigen::Matrix<ceres::Jet<T,N>,R,C>& val) { UNUSED(val); return Eigen::Matrix<double,R,C>(1337); } // ignore Jets

    template <int R, int C, typename T, int N>
    Eigen::Matrix<float,R,C> toFloatV(const Eigen::Matrix<ceres::Jet<T,N>,R,C>& val) { UNUSED(val); return Eigen::Matrix<float,R,C>(1337); } // ignore Jets

    // double

    double toDouble(const double& val);
    float  toFloat(const double& val);

    template <int R, int C>
    Eigen::Matrix<double,R,C> toDoubleV(const Eigen::Matrix<double,R,C>& val) { return val; }

    template <int R, int C>
    Eigen::Matrix<float,R,C> toFloatV(const Eigen::Matrix<double,R,C>& val) { return val.template cast<float>(); }


    // Motion
    template <typename T>
    Eigen::Matrix<float,6,1> castToFVect(const Motion<T>& m) {
        Eigen::Matrix<float,6,1> t; t <<
            CeresCaster::toFloat(m.velocity.x()),
            CeresCaster::toFloat(m.velocity.y()),
            CeresCaster::toFloat(m.velocity.z()),
            CeresCaster::toFloat(m.rotation.x()),
            CeresCaster::toFloat(m.rotation.y()),
            CeresCaster::toFloat(m.rotation.z());
        return t;
    }

    template <typename T>
    Eigen::Matrix<double,6,1> castToDVect(const Motion<T>& m) {
        Eigen::Matrix<double,6,1> t; t <<
            CeresCaster::toDouble(m.velocity.x()),
            CeresCaster::toDouble(m.velocity.y()),
            CeresCaster::toDouble(m.velocity.z()),
            CeresCaster::toDouble(m.rotation.x()),
            CeresCaster::toDouble(m.rotation.y()),
            CeresCaster::toDouble(m.rotation.z());
        return t;
    }


    template <typename T>
    Eigen::Matrix<float,6,1> castToFVect(const Eigen::Matrix<T,6,1>& m) {
        Eigen::Matrix<float,6,1> t; t <<
            CeresCaster::toFloat(m[0]),
            CeresCaster::toFloat(m[1]),
            CeresCaster::toFloat(m[2]),
            CeresCaster::toFloat(m[3]),
            CeresCaster::toFloat(m[4]),
            CeresCaster::toFloat(m[5]);
        return t;
    }

    template <typename T>
    Eigen::Matrix<double,6,1> castToDVect(const Eigen::Matrix<T,6,1>& m) {
        Eigen::Matrix<double,6,1> t; t <<
            CeresCaster::toDouble(m[0]),
            CeresCaster::toDouble(m[1]),
            CeresCaster::toDouble(m[2]),
            CeresCaster::toDouble(m[3]),
            CeresCaster::toDouble(m[4]),
            CeresCaster::toDouble(m[5]);
        return t;
    }
};

/*
template <typename From, typename To>
struct CeresCaster
{
    typedef From _from_t;
    typedef To   _to_t;
    static To cast(const From& x) { UNUSED(x); std::cout << "G"; INFO; return To(1337); }
};

template <typename To>
struct CeresCaster<Eigen::Matrix<To,2,1>, To>
{
    typedef Eigen::Matrix<To,2,1> _from_t;
    typedef To   _to_t;
    static To cast(const Eigen::Matrix<To,2,1>& x) { INFO; return (To)x; }
};

template <typename To>
struct CeresCaster<double, To>
{
    typedef double _from_t;
    typedef To   _to_t;
    static To cast(const double& x) { INFO; return (To)x; }
};

template <typename To>
struct CeresCaster<float, To>
{
    typedef float _from_t;
    typedef To   _to_t;
    static To cast(const float& x) { INFO; return (To)x; }
};

template <>
struct CeresCaster<Eigen::Vector2f, Eigen::Vector2f>
{
    typedef Eigen::Vector2f _from_t;
    typedef Eigen::Vector2f _to_t;
    static Eigen::Vector2f cast(const Eigen::Vector2f& x) { INFO; return x; }
};

#define INT(x)   CeresCaster<Scalar, int>::cast(x)
#define FLOAT(x) CeresCaster<Scalar, float>::cast(x)
#define VEC2F(x) CeresCaster<Eigen::Matrix<Scalar,2,1>, Eigen::Vector2f>::cast(x)
*/

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct UnitNormVectorAddition
{

    template <typename Scalar>
    bool operator()(const Scalar* x, const Scalar* delta, Scalar* x_plus_delta) const {

        Scalar sum = Scalar(0);
        for (size_t i = 0; i < 6; i++) {
            Scalar s = x[i] + delta[i];
            sum += s * s;
            x_plus_delta[i] = s;
        }


        sum = Scalar(1)/ceres::sqrt(sum);

        for (size_t i = 0; i < 6; i++) {
            x_plus_delta[i] *= sum;
        }

        return true;
    }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define AssertItsNotFloat(T) static_assert(!std::is_same<T, float>::value, "Type cannot be float.");

////////////////////////////////////////////////////////////////////////////////
/*
template <typename T, int N>
std::ostream& operator<<(std::ostream& os, const ceres::Jet<T,N>& jet)
{
    os << "Jet<" << type_name<T>() << ", " << N << ">(" << jet.a << ")";
    return os;
}
*/
////////////////////////////////////////////////////////////////////////////////

template <int R, int C, typename T, int N>
std::ostream& operator<<(std::ostream& os, const Eigen::Matrix<ceres::Jet<T,N>,R,C>& jetV)
{
    os << "Eigen::Matrix<Jet, " << R << ", " << C << ">( " << std::endl;

    for (size_t r = 0; r < R; r++) {
        os << "\t";
        for (size_t c = 0; c < C; c++) {
            os << jetV(r,c) << " ";
            //os << "Jet<" << type_name<T>() << ", " << N << ">(" << jet.a << ")";
        }
        os << std::endl;
    }

    os << " )" << std::endl;
    return os;
}

////////////////////////////////////////////////////////////////////////////////


#endif /* end of include guard: CERES_H_DPURA6XC */
