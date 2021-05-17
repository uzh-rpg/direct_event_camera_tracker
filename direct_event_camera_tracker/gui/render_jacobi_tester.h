#ifndef RENDER_JACOBI_TESTER_H_TDLPXHQG
#define RENDER_JACOBI_TESTER_H_TDLPXHQG

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <ceres/cubic_interpolation.h>
#include "sophus/se3.hpp"

#include "point.h"
#include "gui/ImageLabel.h"
#include "opengl/world_renderer.h"
#include "core/camera_intrinsics.h"
#include "optimization/opt_rerender.h"

#include "ui_maingui.h"

#include <QDialog>
#include <QVector>
#include <QGridLayout>
#include <QSpinBox>
#include <QSizePolicy>
#include <QProgressDialog>
#include <QElapsedTimer>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QCheckBox>

#include <qcustomplot.h>

class RenderJacobiTester : public QDialog
{
    typedef Eigen::Matrix<double,6,1> Vec6;

    Q_OBJECT
public:
    RenderJacobiTester(
            const Posef& T_WC_center,
            WorldRenderer& world_renderer,
            const CameraIntrinsics& camera,
            const Ui::MainWindow& ui,
            QWidget* parent = 0);

    void plot_line(const std::vector<QCustomPlot*> plots, double slope, double ycenter);

    Vec6 calc_jacobian(const Keyframe& keyframe, int x, int y);

protected slots:
    void imageHover(double x, double y);

    void renderPerturbedImgs();

    void scalePlotsIdentically(const std::vector<QCustomPlot*> plots);

    void on_smoothing_slider_changed(int val);

private:
    void init_plots(std::vector<QCustomPlot*>& plots, QColor color, int row_offset);

    const Posef& T_WC_center;
    WorldRenderer& world_renderer;
    const CameraIntrinsics& camera;

    const int blur;

    // use some setting from main window
    const Ui::MainWindow& ui;

    QSlider* ui_num_smoothing;
    QLabel*  smoothing_lbl;
    QCheckBox* ui_use_2nd_deriv;

    Keyframe kf;
    ImageLabel* rendered_img;


    Vec6 perturbation;
    std::vector<std::vector<Keyframe>> perturbed_imgs;

    std::vector<QCustomPlot*> err_plots;
    std::vector<QCustomPlot*> J_plots;
};


#endif /* end of include guard: RENDER_JACOBI_TESTER_H_TDLPXHQG */
