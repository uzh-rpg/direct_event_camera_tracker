#ifndef GENERAL_JACOBI_TESTER_H_AOCOZUXP
#define GENERAL_JACOBI_TESTER_H_AOCOZUXP

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <ceres/cubic_interpolation.h>
#include <algorithm>
#include "sophus/se3.hpp"

#include "point.h"
#include "gui/ImageLabel.h"
#include "opengl/world_renderer.h"
#include "core/camera_intrinsics.h"
#include "optimization/optimization.h"

#include "ui_maingui.h"

#include <QDialog>
#include <QVector>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QSizePolicy>
#include <QProgressDialog>
#include <QElapsedTimer>
#include <QCheckBox>

#include <qcustomplot.h>

////////////////////////////////////////////////////////////////////////////////

struct PlotPointData {
    size_t len; // number of residuals
    double xval;
    double* residual;
    double* jacobians[2]; // array[2][blocksize*dim]
    double* jacobians_data; // array[len*dim]

    PlotPointData(size_t N, double xval)
        : len(N), xval(xval),
        residual(new double[N]),
        jacobians_data(new double[N*12])
    {
        // initialize arrays with obviously wrong values, so they stand out if something goes wrong
        std::fill_n(residual, N, -666);
        std::fill_n(jacobians_data, N*12, -1337);

        // init jacobians array of pointers to array
        jacobians[0] = &jacobians_data[0];
        jacobians[1] = &jacobians_data[6*N];

        jacobians[0][0] = 4444;
        jacobians[1][0] = 4444;
    }

    ~PlotPointData() {
        if (residual) delete[] residual;
        if (jacobians_data) delete[] jacobians_data;
    }

    // no copies please!
    PlotPointData(const PlotPointData& other) = delete;
    void operator=(const PlotPointData& other) = delete;

    // but allow move
    PlotPointData(PlotPointData&& other)
        : len(other.len), residual(other.residual), jacobians_data(other.jacobians_data)
    {
        // transfer ownership of pointers
        other.residual = nullptr;
        other.jacobians_data = nullptr;
        jacobians[0] = other.jacobians[0];
        jacobians[1] = other.jacobians[1];
    }
};

////////////////////////////////////////////////////////////////////////////////

struct Plot {
    QCustomPlot* plt;
    size_t attr_idx; // which exact parameter of the 12
    size_t samples;
    std::vector<PlotPointData> data;

    Plot(size_t attr_idx, size_t samples, size_t N, QWidget* parent)
        : plt(new QCustomPlot(parent)), attr_idx(attr_idx), samples(samples)
    {
        for (size_t i = 0; i < samples; i++) {
            data.push_back(PlotPointData(N, 0));
        }

        plt->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        plt->addGraph(); // error
        plt->addGraph(); // derivative of error
        plt->addGraph(); // tangent
        plt->addGraph(); // derivative

        // err derivative
        plt->graph(1)->setPen(QPen(Qt::red));

        // tangent
        QPen tangent_pen;
        tangent_pen.setColor(QColor(155,155,255)); // light blue
        tangent_pen.setStyle(Qt::DotLine);
        tangent_pen.setWidth(3);
        plt->graph(2)->setPen(tangent_pen);

        // derivative (jacobi)
        QPen deriv_pen(QColor(255,0,0,100)); // transparent red
        deriv_pen.setWidth(4);
        deriv_pen.setStyle(Qt::DotLine);
        plt->graph(3)->setPen(deriv_pen);

        plt->yAxis->setLabel("residual (||err||)");

        // add title
        plt->plotLayout()->insertRow(0);
        plt->plotLayout()->addElement(0, 0, new QCPPlotTitle(plt, Statef::get_attr_names()[attr_idx]));
    }

    // no copies please!
    Plot(const Plot& other) = delete;
    void operator=(const Plot& other) = delete;
};

////////////////////////////////////////////////////////////////////////////////

class GeneralJacobiTester : public QDialog
{
    Q_OBJECT
public:
    GeneralJacobiTester(
            std::unique_ptr<IOptimization>&& optim,
            const Ui::MainWindow& ui,
            QWidget* parent = 0);

    ~GeneralJacobiTester();

signals:
    void stateHovered (const Statef& s);

public slots:
    void add_point(const PlotPoint& s, bool fading = false);
    void clear_points();

    void plot_total_err_for_all();
    void plot_mouse_move(Plot& plt, QMouseEvent* event);

    void show_specific_pixel(float x, float y);

    void scale_to_err_only();
    void scale_to_jacobi_only();

private:

    void calculate_errors(Plot& plt);

    void plot(Plot& plt, boost::function<double(Plot&, int)> f, size_t plot_nr = 0);
    void plot_total_err(Plot& plt);
    void plot_single_px(Plot& plt, size_t x, size_t y);

    void plot_tangent(Plot& plt, float slope, float ycenter);
    void plot_numeric_diff(Plot& plt, const std::vector<double>& vals);

    // use some settings from main window
    const Ui::MainWindow& mainui;
    QLabel* ui_cursor_x;
    QLabel* ui_cursor_y;

    std::unique_ptr<IOptimization> optim;


    Eigen::Matrix<double,6,1> state_pos;
    Eigen::Matrix<double,6,1> state_vel;
    ceres::CostFunction* cost_func;

    std::vector<Plot*> plots;
};


#endif /* end of include guard: GENERAL_JACOBI_TESTER_H_AOCOZUXP */
