#include "utils.h"

using namespace cv;
using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
/**
 *
 * see also https://docs.opencv.org/3.2.0/d2/d2c/tutorial_sobel_derivatives.html
 *
 * TODO: this function could probably be optimized quite a lot by not having to
 * create all these temporary matrices.
 */
void image_gradient(const cv::Mat& src, cv::Mat& dst)
{
    assert(!src.empty());

    // convert input to grayscale if necessary
    if (src.channels() == 3) {
        cvtColor(src, src, COLOR_BGR2GRAY);
    } else {
        assert(src.channels() == 1);
    }

    // reduce noice a bit
    // -> Sobel operator already includes a gaussian smoothing term
    //GaussianBlur(src_gray, src_gray, Size(3,3), 0, 0, BORDER_ISOLATED);


    Mat grad_x, grad_y;

    // calculate actual gradient (using a normalized kernel)
    // normalization factor is sum of elements for the smoothing part (16 for SCHARR)
    // times 1/2 for the finite differences due to employing central differences 
    // (or for the full kernel: sum(abs(elements))
    Sobel(src, grad_x, CV_64F, 1, 0, CV_SCHARR, 1/32.0f, 0, BORDER_REPLICATE);
    Sobel(src, grad_y, CV_64F, 0, 1, CV_SCHARR, 1/32.0f, 0, BORDER_REPLICATE);

    // merge gradients into single image with two channels
    Mat mats[2] = { grad_x, grad_y };
    merge(mats, 2, dst);
}

////////////////////////////////////////////////////////////////////////////////

void gradient_of_gradient(const cv::Mat& src, cv::Mat& dst)
{
    cv::Mat grad_x, grad_y;

    // TODO: we can optimize away one sobel, as d/dy*d/dx == d/dx*d/dy

    Sobel(src, grad_x, CV_64F, 1, 0, CV_SCHARR, 1/32.0f, 0, cv::BORDER_REPLICATE);
    Sobel(src, grad_y, CV_64F, 0, 1, CV_SCHARR, 1/32.0f, 0, cv::BORDER_REPLICATE);

    cv::Mat grad_x_arr[2];
    cv::Mat grad_y_arr[2];
    cv::split(grad_x, grad_x_arr);
    cv::split(grad_y, grad_y_arr);

    // grad_x and _y are now CV_64FC2:
    cv::Mat all_grads[] = {
        grad_x_arr[0], // grad_x[0] = d/dx d/dx I
        grad_y_arr[0], // grad_y[0] = d/dy d/dx I
        grad_x_arr[1], // grad_x[1] = d/dx d/dy I
        grad_y_arr[1], // grad_y[1] = d/dy d/dy I
    };

    cv::merge(all_grads, 4, dst);
}

////////////////////////////////////////////////////////////////////////////////

// calculate angle and magnitude of derivative for visualization
void visualize_gradient(const cv::Mat& src, cv::Mat& dst)
{
    // split input into x and y gradients
    vector<Mat> gradients;
    split(src, gradients);

    // convert to float (cvtColor cannot handle floats :()
    gradients[0].convertTo(gradients[0], CV_32FC1);
    gradients[1].convertTo(gradients[1], CV_32FC1);

    // calculate angle (in degree) and magnitude
    Mat angle; // 0 to 360
    Mat magnitude; // 0 to ???
    cartToPolar(gradients[0], gradients[1], magnitude, angle, true);

    // scale magnitude to [0,1]
    double max_mag, min_mag;
    minMaxLoc(magnitude, &min_mag, &max_mag);
    magnitude /= max_mag;

    // Hue: 0-360 (float)
    // Saturation: 0-1 (float)
    // Value: 0-1 (float)

    // initialize 'value' with 1
    Mat value(src.size(), CV_32FC1, Scalar(1));

    Mat hsv_in[] = { angle, magnitude, value }; // Hue, Saturation, Value
    Mat hsv;
    merge(hsv_in, 3, hsv);

    cvtColor(hsv, dst, COLOR_HSV2BGR);
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat visualize_correlation(const cv::Mat& src)
{
    assert(src.type() == CV_64FC1);

    double min_val, max_val;
    cv::minMaxLoc(src, &min_val, &max_val);

    // calculate max(abs(I))
    if (-min_val > max_val) {
        max_val = -min_val;
    }

    cv::Mat dst = cv::Mat(src.size(), CV_8UC3, cv::Scalar(0,0,0));

    // yeah, we could implement this using convertTo and other functions, but this was easier to think about ;)
    for (size_t y = 0; y < src.rows; y++) {
        for (size_t x = 0; x < src.cols; x++) {

            double v = 255 * src.at<double>(y,x) / max_val;
            //cout << src.at<double>(y,x) << " / " << max_val << " * 255 = " << v << endl;

            if (v >= 0) {
                dst.at<cv::Vec3b>(y,x) = cv::Vec3b(0,v,0); // BGR!
            } else {
                dst.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,-v); // BGR!
            }
        }
    }

    return dst;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat plot_gradient_legend()
{
    cv::Mat grad(300, 300, CV_64FC2, cv::Scalar(0,0));

    // polar plot
    for (int a = 0; a < 3600; a++) {
        for (int r = 40; r < 130; r++) {
            double angle = a / 3600.0 * 2 * M_PI;
            Eigen::Vector2d cart_pos(r * cos(angle), r * sin(angle));
            grad.at<Eigen::Vector2d>(
                    (int)cart_pos.y()+grad.rows/2,
                    (int)cart_pos.x()+grad.cols/2) = cart_pos.normalized();
        }
    }

    cv::Mat viz;
    visualize_gradient(grad, viz);
    return viz;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream & operator<<(std::ostream &out, const QSizeF& size)
{
    out << "[ " << size.width() << " x " << size.height() << " ]";
    return out;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream & operator<<(std::ostream &out, const QSize& size)
{
    out << "[ " << size.width() << " x " << size.height() << " ]";
    return out;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream & operator<<(std::ostream &out, const QPointF& point)
{
    out << point.x() << ", " << point.y();
    return out;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream & operator<<(std::ostream &out, const QPoint& point)
{
    out << point.x() << ", " << point.y();
    return out;
}

////////////////////////////////////////////////////////////////////////////////

void downsample2_img(const cv::Mat& src, cv::Mat& dst)
{
    cv::resize(src, dst, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
}

////////////////////////////////////////////////////////////////////////////////

void minmax_sqnorm(const cv::Mat& src, double& out_min, double& out_max, const cv::Mat& mask)
{
    out_min =  numeric_limits<double>::infinity();
    out_max = -numeric_limits<double>::infinity();

    for (size_t y = 0; y < src.rows; y++) {
        for (size_t x = 0; x < src.cols; x++) {

            // ignore invalid pixels
            if (mask.at<double>(y,x) < 0)
                continue;

            double n = src.at<Vector2d>(y,x).squaredNorm();
            if (out_min > n)
                out_min = n;

            if (out_max < n)
                out_max = n;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

size_t count_above(const cv::Mat& src, const double& thresh, const cv::Mat& mask)
{
    size_t out_count = 0;
    for (size_t y = 0; y < src.rows; y++) {
        for (size_t x = 0; x < src.cols; x++) {

            // ignore invalid pixels
            if (mask.at<double>(y,x) < 0)
                continue;

            double n = src.at<Vector2d>(y,x).squaredNorm();
            if (n >= thresh) {
                out_count++;
            }
        }
    }

    return out_count;
}

////////////////////////////////////////////////////////////////////////////////
// we could implement something like quickselect here, but I this approximation
// should be good enough
//
// distribution of gradients seems to be approximately logarithmic!
#if 1
double find_threshold(const cv::Mat& src, const double& fraction, size_t& out_count, const cv::Mat& mask)
{
    // make sure we get 100% if 100% were requested
    if (fraction > 0.99) {
        out_count = src.rows * src.cols;
        return 0;
    }

    // this function is only ever used for gradients ;)
    assert(src.type() == CV_64FC2);

    assert(fraction > 0);
    assert(fraction <= 1);

    double min_n, max_n;
    minmax_sqnorm(src, min_n, max_n, mask);

    if (min_n <= 0)
        min_n = 1e-7; // prevent -inf in log // TODO: this number is actually quite critical -.-

    // we get the min/max SQUARE norm
    // also, we're dealing with something approximately logarithmic/exponential
    double log_min_n = log(sqrt((double) min_n));
    double log_max_n = log(sqrt((double) max_n));

    // guess our threshold to be somewhere in the middle in log space ;)
    double thresh = (log_max_n - log_min_n) * (1-fraction) + log_min_n;
    //cout << "interm: " << thresh << endl;
    thresh = pow(exp(thresh), 2);

    // check how close we got
    out_count = count_above(src, thresh, mask);

    /*
    cout << "min: " << min_n << " max: " << max_n << endl;
    cout << "min: " << log(sqrt(min_n)) << " max: " << log(sqrt(max_n)) << endl;
    cout << " -> threshold = " << thresh << endl;
    cout << "Tried do find threshold to select " << fraction*100 << "% of all pixels, actually got " << out_count/double(src.size().area())*100 << "% (" << out_count << " of " << src.size().area() << ") of em'" << endl;
    */

    return thresh;
}
#else
////////////////////////////////////////////////////////////////////////////////
/*
 This is a bit more accurate but quite a lot more complicated...
 */
double find_threshold(const cv::Mat& src, const double& fraction, size_t& out_count)
{
    const size_t order = 2; // quadratic fit
    const size_t N = 10;
    Eigen::Matrix<double, N, 1> counts(N);
    Eigen::Matrix<double, N, 1> threshs(N);

    // get extreme values
    double min_n, max_n;
    minmax_sqnorm(src, min_n, max_n);

    if (min_n <= 0)
        min_n = 1;


    min_n = log(sqrt(min_n));
    max_n = log(sqrt(max_n));

    // create a corse histogram
    threshs[0]   = min_n; counts[0]   = src.size().area();
    threshs[N-1] = max_n; counts[N-1] = 0;

    //cout << 0 << ": count for " << threshs[0] << ": " << counts[0] << endl;
    for (size_t i = 1; i < N-1; i++) {
        threshs[i] = (max_n-min_n) * (i/double(N-1)) + min_n;
        counts[i] = count_above(src, pow(exp(threshs[i]),2));
        //cout << i << ": count for " << threshs[i] << ": " << counts[i] << endl;
    }
    //cout << N-1 << ": count for " << threshs[N-1] << ": " << counts[N-1] << endl;

    // fit a polynomial
    Eigen::Matrix<double, order+1, 1> poly = polyfit<double, N, order>(counts, threshs);

    /*
    cout << "poly: " << endl;
    cout << poly << endl;

    cout << "counts:" << endl;
    cout << counts << endl;

    cout << "threshs:" << endl;
    cout << threshs << endl;
    */

    // evaluate it
    double x = fraction * src.size().area();
    double thresh = 0;
    for (size_t i = 0; i < poly.rows(); i++) {
        thresh += pow(x, i) * poly[i];
    }

    //cout << "found threshold: " << thresh << endl;
    thresh = pow(exp(thresh), 2);
    //cout << " --> " << thresh << endl;

    out_count = count_above(src, thresh);

    //cout << "this gives " << out_count << " pixels = " << out_count / double(src.size().area()) * 100 << "%" << endl;
    //cout << "target count: " << (fraction * src.size().area()) << endl;

    return thresh;
}
#endif
////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2i toVec2i(const cv::Size& obj)
{
    return Eigen::Vector2i(obj.width, obj.height);
}

////////////////////////////////////////////////////////////////////////////////

std::string time_to_str(const ros::Time& t)
{
    ostringstream s;
    s << t;
    return s.str();
}

////////////////////////////////////////////////////////////////////////////////

QVector<double> moving_average(const QVector<double>& data, const int window_size)
{
    const size_t N = data.length();
    QVector<double> result(N);

    for (int i = 0; i < N; i++) {

        double sum = 0;
        for (int k = -window_size; k <= window_size; k++) {
            double v;
            if      (i+k <  0) { v = data.front(); }
            else if (i+k >= N) { v = data.back(); }
            else               { v = data[i+k]; }
            sum += v;
        }

        result[i] = sum / (window_size*2+1);
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

