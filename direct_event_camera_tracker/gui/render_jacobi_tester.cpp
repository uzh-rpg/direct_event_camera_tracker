#include "render_jacobi_tester.h"

using namespace std;

// indexes for QCustomPlot graphs
enum GraphIdx {
    // err plots
    GRAPH_MEAS_VALUE_X = 0,
    GRAPH_MEAS_VALUE_Y = 1,
    GRAPH_SMOOTH_MEAS_VALUE_X = 2,
    GRAPH_SMOOTH_MEAS_VALUE_Y = 3,
    GRAPH_TANGENT_X = 4,
    GRAPH_TANGENT_Y = 5,

    // J_plots
    GRAPH_ANALYTIC_DERIV_X = 0,
    GRAPH_ANALYTIC_DERIV_Y = 1,
    GRAPH_NUMERIC_DERIV_X = 2,
    GRAPH_NUMERIC_DERIV_Y = 3,
};

////////////////////////////////////////////////////////////////////////////////

RenderJacobiTester::RenderJacobiTester(
            const Posef& T_WC_center,
            WorldRenderer& world_renderer,
            const CameraIntrinsics& camera,
            const Ui::MainWindow& ui,
            QWidget* parent)
    : QDialog(parent),
    T_WC_center(T_WC_center),
    world_renderer(world_renderer),
    camera(camera),
    blur(0),
    ui(ui)
{
    perturbation <<
        ui.dim1_dev->value(),
        ui.dim1_dev->value(),
        ui.dim1_dev->value(),
        ui.dim2_dev->value(),
        ui.dim2_dev->value(),
        ui.dim2_dev->value();

    QGridLayout* layout = new QGridLayout(this);

    QVBoxLayout* vert_layout = new QVBoxLayout();
    rendered_img = new ImageLabel(this);
    vert_layout->addWidget(rendered_img);

    ImageLabel* rendered_img_grad = new ImageLabel(this);
    vert_layout->addWidget(rendered_img_grad);

    ui_num_smoothing = new QSlider(Qt::Horizontal, this);
    ui_num_smoothing->setMinimum(0);
    ui_num_smoothing->setMaximum(ui.samples_count->value()/4);
    connect(ui_num_smoothing, &QSlider::valueChanged, this, &RenderJacobiTester::on_smoothing_slider_changed);
    vert_layout->addWidget(ui_num_smoothing);

    smoothing_lbl = new QLabel(this);
    on_smoothing_slider_changed(0);
    vert_layout->addWidget(smoothing_lbl);

    ui_use_2nd_deriv = new QCheckBox("check second derivative instead of first");
    vert_layout->addWidget(ui_use_2nd_deriv);

    layout->addLayout(vert_layout, 0, 0, 4, 1);

    setLayout(layout);

    kf = world_renderer.renderPose(T_WC_center);

    // smooth image a bit
    kf.blur(blur);

    rendered_img->setPixmap(kf.intensity);
    rendered_img_grad->setPixmap(kf.gradient);

    rendered_img->setMouseTracking(true);
    connect(rendered_img, &ImageLabel::pixelHovered, this, &RenderJacobiTester::imageHover);

    init_plots(err_plots, Qt::blue, 0);
    init_plots(J_plots,   Qt::red,  1);

    renderPerturbedImgs();
}

////////////////////////////////////////////////////////////////////////////////

void RenderJacobiTester::init_plots(std::vector<QCustomPlot*>& plots, QColor color, int row_offset)
{
    QGridLayout* layout = static_cast<QGridLayout*>(this->layout());

    plots.resize(6);
    for (size_t i = 0; i < 6; i++) {
        plots[i] = new QCustomPlot(this);
        plots[i]->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        plots[i]->addGraph(); // measured value x
        plots[i]->addGraph(); // measured value y
        plots[i]->addGraph(); // smoothed measured value x
        plots[i]->addGraph(); // smoothed measured value y

        if (row_offset == 0) { // not very clean, but wayne
            // set colors
            plots[i]->graph(GRAPH_MEAS_VALUE_X)->setPen(QPen(color));
            plots[i]->graph(GRAPH_MEAS_VALUE_Y)->setPen(QPen(color));


            QPen smooth_pen(color);
            smooth_pen.setWidth(2);
            plots[i]->graph(GRAPH_SMOOTH_MEAS_VALUE_X)->setPen(smooth_pen);
            plots[i]->graph(GRAPH_SMOOTH_MEAS_VALUE_Y)->setPen(smooth_pen);

            plots[i]->yAxis->setLabel("intensity");
        } else {
            plots[i]->graph(GRAPH_NUMERIC_DERIV_X)->setPen(QPen(color));
            plots[i]->graph(GRAPH_NUMERIC_DERIV_Y)->setPen(QPen(color));

            // derivative plots
            QColor color2 = color;
            color2.setAlpha(100);
            QPen deriv_pen(color2);
            deriv_pen.setWidth(4);
            deriv_pen.setStyle(Qt::DotLine);
            plots[i]->graph(GRAPH_ANALYTIC_DERIV_X)->setPen(deriv_pen);

            QPen deriv_peny = deriv_pen;
            deriv_peny.setColor(QColor(0,255,0,255));
            plots[i]->graph(GRAPH_ANALYTIC_DERIV_Y)->setPen(deriv_peny);

            plots[i]->yAxis->setLabel("delta intensity");
        }

        layout->addWidget(plots[i], ((int)i/3)*2+row_offset, i%3+1);
    }

    plots[0]->xAxis->setLabel("translation X deviation");
    plots[1]->xAxis->setLabel("translation Y deviation");
    plots[2]->xAxis->setLabel("translation Z deviation");
    plots[3]->xAxis->setLabel("twist X deviation");
    plots[4]->xAxis->setLabel("twist Y deviation");
    plots[5]->xAxis->setLabel("twist Z deviation");
}

////////////////////////////////////////////////////////////////////////////////

void RenderJacobiTester::imageHover(double x, double y)
{
    int D = ui.samples_count->value();
    int N = D*2 + 1;

    if (kf.getPixel(x,y).depth <= 0) {
        return; // skip invalid pixels
    }

    //Vec6 Jkf = calc_jacobian(kf, int(x), int(y));

    for (int dim = 0; dim < 6; dim++) {
        bool use2nd = ui_use_2nd_deriv->isChecked();

        for (int subdim = 0; subdim < (use2nd?2:1); subdim++) {
            QVector<double> xvals(N), yvals(N);
            QVector<double> dxvals(N-1), dyvals(N-1);
            QVector<double> xvals_interp(N), yvals_interp(N);

            // get intensities from chosen pixel
            for (int i = 0; i < N; i++) {
                xvals[i] = double(i-D)/D * perturbation[dim];

                if (use2nd) {
                    yvals[i] = perturbed_imgs[dim][i].getPixel(x,y).gradient[subdim];
                } else {
                    yvals[i] = perturbed_imgs[dim][i].getPixel(x,y).intensity;
                }
            }

            // smooth values
            QVector<double> yvals_smoothed;
            if (ui_num_smoothing->value() > 0) {
                yvals_smoothed = moving_average(yvals, ui_num_smoothing->value());
            } else {
                yvals_smoothed = yvals;
            }

            // calculate numeric diff
            for (int i = 0; i < N-1; i++) {
                dxvals[i] = (xvals[i+1] + xvals[i])/2;
                dyvals[i] = (yvals_smoothed[i+1] - yvals_smoothed[i])/(xvals[i+1] - xvals[i]);
            }


            /*
               ceres::Grid1D<double> yvals_grid(yvals.data(), 0, yvals.length());
               ceres::CubicInterpolator<ceres::Grid1D<double>> yvals_interpolator(yvals_grid);

               for (int i = 0; i < N; i++) {
               xvals_interp[i] = xvals[i];

               double v; yvals_interpolator.Evaluate(i, &v);
               yvals_interp[i] = v;

               cout << "interpolating at " << i << ": " << xvals_interp[i] << " -> " << v << endl;
               }
               */


            // plot analytic derivative
            QVector<double> Jxvals(N), Jyvals(N);
            for (size_t i = 0; i < N; i++) {
                if (use2nd) {
                    KFRenderingJacobian Jr(perturbed_imgs[dim][i]);

                    double residuals[2];
                    double jacobian[12];
                    double* jacobians = {jacobian};
                    double* params[1];

                    Jr.pos = Eigen::Vector2i(x,y);
                    Jr.Evaluate(params, residuals, &jacobians);
                    Jyvals[i] = jacobian[dim + 6*subdim];

                } else {
                    Vec6 J = calc_jacobian(perturbed_imgs[dim][i], int(x), int(y));
                    Jyvals[i] = J[dim];
                }
                Jxvals[i] = xvals[i];
            }

            // plot measured values
            err_plots[dim]->graph(GRAPH_MEAS_VALUE_X + subdim)       ->setData(QVector<double>(xvals), QVector<double>(yvals));
            err_plots[dim]->graph(GRAPH_SMOOTH_MEAS_VALUE_X + subdim)->setData(QVector<double>(xvals), QVector<double>(yvals_smoothed));

            // plot tangent
            //plot_line(err_plots, Jkf[dim], yvals_smoothed[D]);

            J_plots[dim]->graph(GRAPH_NUMERIC_DERIV_X + subdim) ->setData(dxvals, dyvals);
            J_plots[dim]->graph(GRAPH_ANALYTIC_DERIV_X + subdim)->setData(Jxvals, Jyvals);
        }

        scalePlotsIdentically(err_plots);
        scalePlotsIdentically(J_plots);

        err_plots[dim]->replot();
        J_plots  [dim]->replot();
    }
}

////////////////////////////////////////////////////////////////////////////////

void RenderJacobiTester::renderPerturbedImgs()
{
    int D = ui.samples_count->value();
    int N = D*2 + 1;
    perturbed_imgs.resize(6);

    QProgressDialog progress("rendering a lot of images...", QString(), 0, 6*N, dynamic_cast<QWidget*>(this->parent()));
    progress.setMinimumDuration(0); // show dialog immediately
    progress.setWindowModality(Qt::WindowModal);

    QElapsedTimer timer;
    timer.start();

    for (int dim = 0; dim < 6; dim++) {
        perturbed_imgs[dim].resize(N);

        for (int i = 0; i < N; i++) {
            progress.setValue(dim*N + i);

            Vec6 dx = Vec6::Zero();
            dx[dim] = perturbation[dim] * double(i - D)/D;

            Eigen::Isometry3d T_perturbed = T_WC_center.T();
            T_perturbed *= Sophus::SE3d::exp(dx).matrix();

            perturbed_imgs[dim][i] = world_renderer.renderPose(T_perturbed);

            perturbed_imgs[dim][i].blur(blur);
        }
    }

    long int ns = timer.nsecsElapsed();
    cout << "rendering " << 6*N << " images took " << ns/1000/1000 << " milliseconds, which equals " << double(ns)/(1000*1000*6*N) << " ms per render" << endl;

    progress.close();
}

////////////////////////////////////////////////////////////////////////////////

// draw a line trough [x = 0, y = ycenter] with specific slope
void RenderJacobiTester::plot_line(const std::vector<QCustomPlot*> plots, double slope, double ycenter)
{
    for (int dim = 0; dim < 6; dim++) {
        QVector<double> xvals(2), yvals(2);

        xvals[0] = -perturbation[dim];
        xvals[1] =  perturbation[dim];

        yvals[0] = ycenter - perturbation[dim]*slope;
        yvals[1] = ycenter + perturbation[dim]*slope;

        plots[dim]->graph(GRAPH_TANGENT_X)->setData(xvals, yvals);
    }
}

////////////////////////////////////////////////////////////////////////////////

RenderJacobiTester::Vec6 RenderJacobiTester::calc_jacobian(const Keyframe& keyframe, int x, int y)
{
    Vec6 J;
    Pixelf px = keyframe.getPixel(x,y);

    for (int dim = 0; dim < 6; dim++) {
        Motionf m;

        if (dim < 3) {
            m.velocity[dim] = 1;
        } else {
            m.rotation[dim-3] = 1;
        }

        Eigen::Vector2d flow = px.calc_flow(kf.camera, m);

        J[dim] = -flow.dot( px.gradient );
    }

    return J;
}

////////////////////////////////////////////////////////////////////////////////

void RenderJacobiTester::scalePlotsIdentically(const std::vector<QCustomPlot*> plots)
{
    double max_dy = 0;
    int max_dim = 0;
    for (int dim = 0; dim < 6; dim++) {
        plots[dim]->graph(GRAPH_MEAS_VALUE_X)->rescaleAxes(); // scale so that raw data fits (ignore slope)
        plots[dim]->graph(GRAPH_MEAS_VALUE_Y)->rescaleAxes(true);
        plots[dim]->graph(GRAPH_SMOOTH_MEAS_VALUE_X)->rescaleAxes(true);
        plots[dim]->graph(GRAPH_SMOOTH_MEAS_VALUE_Y)->rescaleAxes(true);

        double s = plots[dim]->yAxis->range().size();
        if (s > max_dy) {
            max_dy = s;
            max_dim = dim;
        }
    }

    // now scale all ranges to have a range of max_dy
    for (int dim = 0; dim < 6; dim++) {
        if (dim != max_dim) {
            plots[dim]->yAxis->setScaleRatio(plots[max_dim]->yAxis);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void RenderJacobiTester::on_smoothing_slider_changed(int val)
{
    if (val > 0) {
        smoothing_lbl->setText("smoothing window: " + QString::number(val*2+1));
    } else {
        smoothing_lbl->setText("smoothing disabled");
    }
}

////////////////////////////////////////////////////////////////////////////////

