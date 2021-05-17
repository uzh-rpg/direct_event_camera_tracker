#include "general_jacobi_tester.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// plot titles for the GUI
const std::string attr_names[] = {
    "Position X",
    "Position Y",
    "Position Z",
    "Twist X",
    "Twist Y",
    "Twist Z",
    "Linear Velocity X",
    "Linear Velocity Y",
    "Linear Velocity Z",
    "Angular Velocity X",
    "Angular Velocity Y",
    "Angular Velocity Z"
};

////////////////////////////////////////////////////////////////////////////////

GeneralJacobiTester::GeneralJacobiTester(
        std::unique_ptr<IOptimization>&& optim,
        const Ui::MainWindow& ui,
        QWidget* parent)
    : QDialog(parent), mainui(ui), optim(std::move(optim)),
    cost_func(this->optim->make_cost_function())
{

    QGridLayout* layout = new QGridLayout(this);

    QVBoxLayout* vert_layout = new QVBoxLayout();

    QPushButton* show_total_err = new QPushButton("show total error", this);
    connect(show_total_err, &QPushButton::clicked, this, &GeneralJacobiTester::plot_total_err_for_all);
    vert_layout->addWidget(show_total_err);

    QPushButton* rescale = new QPushButton("scale to error", this);
    connect(rescale, &QPushButton::clicked, this, &GeneralJacobiTester::scale_to_err_only);
    vert_layout->addWidget(rescale);

    rescale = new QPushButton("scale to derivative", this);
    connect(rescale, &QPushButton::clicked, this, &GeneralJacobiTester::scale_to_jacobi_only);
    vert_layout->addWidget(rescale);


    ui_cursor_x = new QLabel("X: ???", this); vert_layout->addWidget(ui_cursor_x);
    ui_cursor_y = new QLabel("Y: ???", this); vert_layout->addWidget(ui_cursor_y);


    ImageLabel* img_label = new ImageLabel(this);
    img_label->setPixmap(this->optim->getParams().keyframe.intensity);
    img_label->setMouseTracking(true);
    connect(img_label, &ImageLabel::pixelHovered, this, &GeneralJacobiTester::show_specific_pixel);
    vert_layout->addWidget(img_label);

    layout->addLayout(vert_layout, 0, 0, 4, 1);

    QProgressDialog progress("calculating residuals and jacobians", QString(), 0, 12, dynamic_cast<QWidget*>(this->parent()));
    progress.setMinimumDuration(0); // show dialog immediately
    progress.setWindowModality(Qt::WindowModal);

    QElapsedTimer timer;
    timer.start();


    const std::vector<ceres::int32> p = this->cost_func->parameter_block_sizes();
    cout << "parameter block sizes: number of blocks = " << p.size() << endl;
    for (ceres::int32 i: p) {
        cout << " >  " << i << endl;
    }

    cout << "number of residuals: " << cost_func->num_residuals() << endl;

    for (size_t attr_idx = 0; attr_idx < 12; attr_idx++) {
        progress.setValue(attr_idx);

        plots.push_back(new Plot(
                    attr_idx,
                    mainui.samples_count->value(),
                    cost_func->num_residuals(),
                    this));

        Plot* plt = plots.back();

        calculate_errors(*plt);
        plot_total_err(*plt);

        layout->addWidget(plt->plt, attr_idx/3, attr_idx%3+1); // first colum is occupied by image

        connect(plt->plt, &QCustomPlot::mouseMove, [plt, this](QMouseEvent* e){ this->plot_mouse_move(*plt, e); });
    }

    long int ns = timer.nsecsElapsed();
    long int N = 12*mainui.samples_count->value();
    cout << "rendering " << N << " datapoints took " << ns/1000/1000 << " milliseconds, which equals " << double(ns)/(1000*1000*N) << " ms per point" << endl;

    setLayout(layout);
    progress.close();
}

////////////////////////////////////////////////////////////////////////////////

GeneralJacobiTester::~GeneralJacobiTester()
{
    //if (state_pos) { delete[] state_pos; }
    //if (state_vel) { delete[] state_vel; }
    if (cost_func) { delete cost_func; }
    if (optim)     { optim.release(); } // not sure why this is necessary, but it will crash otherwise

    for (Plot* p: plots) {
        delete p;
    }

    plots.clear();
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::calculate_errors(Plot& plt)
{
    float deviation = plt.attr_idx < 6 ? mainui.dim1_dev->value() : mainui.dim2_dev->value();

    //cout << "calculating error for attr " << Statef::get_attr_names()[plt.attr_idx].toStdString()
         //<< " (nr " << plt.attr_idx << ") at dim " << plt.pose_or_motion << " and idx " << plt.pm_local_idx << endl;

    double* params[] {
        state_pos.data(),
        state_vel.data()
    };

    for (int i = 0; i < plt.samples; i++) {

        //cout << "    - sample " << i << " of " << plt.samples << endl;

        optim->init_optimization(state_pos, state_vel);

        float val = float(i)/plt.samples*deviation*2 - deviation;
        if (plt.attr_idx < 6) {
            state_pos[plt.attr_idx] += val;
            plt.data[i].xval = state_pos[plt.attr_idx];
        } else {
            state_vel[plt.attr_idx-6] += val;
            plt.data[i].xval = state_vel[plt.attr_idx-6];
        }

        cost_func->Evaluate(params, plt.data[i].residual, plt.data[i].jacobians);

        /*
        cout << plt.attr_idx << " sample " << i << " jacobian of px 0 and attr " << plt.attr_idx << " = " << plt.data[i].jacobians[plt.pose_or_motion][plt.pm_local_idx] << endl;
        cout << " > raw jacobian data: (pose_or_motion = " << plt.pose_or_motion << ", param count = " << plt.data[0].dim << ")" << endl;
        for (size_t k = 0; k < plt.data[i].len; k++) {
            cout << "  - px " << k << ": ";

            for (size_t d = 0; d < 6; d++) {
                cout << plt.data[i].jacobians[0][k*6+d] << "\t ";
            }

            cout << "|";

            for (size_t d = 0; d < 6; d++) {
                cout << plt.data[i].jacobians[1][k*6+d] << "\t ";
            }

            cout << endl;
        }
        */
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot(Plot& plt, boost::function<double(Plot&, int)> f, size_t plot_nr)
{
    QVector<double> xvals(plt.samples), yvals(plt.samples);

    for (int i = 0; i < plt.samples; i++) {

        xvals[i] = plt.data[i].xval;
        double err = f(plt, i);

        if (isnan(err) || isinf(err)) {
            cerr << "WARNING: Invalid correlation " << i << " / " << plt.samples << " at position: " << err << endl;
            //cerr << T_CK;
            yvals[i] = 0;
        } else {
            yvals[i] = err;
            //cout << dim << ", " << idx << ", " << i << " = " << val << ": " << err << endl;
        }
    }

    /*
    cout << "got values: " << endl;
    for (int i = 0; i < plt.samples; i++) {
        cout << i << ": " << xvals[i] << " -> " << yvals[i] << endl;
    }
    */

    plt.plt->graph(plot_nr)->setData(xvals, yvals);
    plt.plt->graph(plot_nr)->rescaleAxes(plot_nr > 0);
    plt.plt->replot();
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_tangent(Plot& plt, float slope, float ycenter)
{
    QVector<double> xvals(3), yvals(3);

    xvals[0] = plt.data[0].xval;
    xvals[1] = plt.data[plt.samples/2].xval;
    xvals[2] = plt.data[plt.samples-1].xval;

    yvals[0] = ycenter + (xvals[0]-xvals[1])*slope;
    yvals[1] = ycenter;
    yvals[2] = ycenter + (xvals[2]-xvals[1])*slope;

    /*
    cout << "plotting tangent of attr " << Statef::get_attr_names()[plt.attr_idx].toStdString() << " with slope = " << slope << " and ycenter = " << ycenter << ":\n";
    cout << "xvals[0]: " << xvals[0] << " \t yvals[0]: " << yvals[0] << endl;
    cout << "xvals[1]: " << xvals[1] << " \t yvals[1]: " << yvals[1] << endl;
    cout << "xvals[2]: " << xvals[2] << " \t yvals[2]: " << yvals[2] << endl;
    */

    plt.plt->graph(2)->setData(xvals, yvals);

    plt.plt->replot();
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_total_err(Plot& plt)
{
    // residual
    vector<double> total_residual(plt.samples);

    plot(plt, [&](Plot& p, int sample){
        double err = 0;
        for (size_t k = 0; k < p.data[sample].len; k++) {
            double v = p.data[sample].residual[k];
            err += v * v;
        }
        total_residual[sample] = err;
        return err;
    }, 0);

    // manual (numeric) derivative from residual
    plot_numeric_diff(plt, total_residual);

    // derivative from Jacobian
    plot(plt, [](Plot& p, int sample) {
        double J = 0;
        for (size_t k = 0; k < p.data[sample].len; k++) {
            // err = sum(x_i^2)
            // d/dx err = 2 sum(x_i * d/dx x_i)
            size_t J_idx  = p.attr_idx < 6 ? 0 : 1;

            double v = 2 * p.data[sample].residual[k] * p.data[sample].jacobians[J_idx][k*6+p.attr_idx%6];
            J += v;
        }
        return J;
    }, 3);

    plt.plt->replot();
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_single_px(Plot& plt, size_t x, size_t y)
{
    size_t imwidth = optim->getParams().eventframe.img.cols;
    // this isn't actually accurate, as invalid pixels are skipped
    size_t k = y*imwidth + x;

    vector<double> residual(plt.samples);

    if (k >= plt.data[0].len) {
        return;
    }

    // residual itself
    plot(plt, [k, &residual](Plot& p, int sample) {
            residual[sample] = p.data[sample].residual[k];
            return p.data[sample].residual[k];
            }, 0);

    // manual (numeric) derivative from residual
    plot_numeric_diff(plt, residual);

    // tangent to residual at center
    int idx = plt.samples/2;
    double dfdx = plt.data[idx].jacobians[plt.attr_idx<6?0:1][k*6+plt.attr_idx%6];
    plot_tangent(plt, dfdx, plt.data[idx].residual[k]);

    // derivative from jacobian
    plot(plt, [k](Plot& p, int sample) {
            double dfdx = p.data[sample].jacobians[p.attr_idx<6?0:1][k*6+p.attr_idx%6];
            return dfdx;
            }, 3);

    plt.plt->replot();
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::add_point(const PlotPoint& s, bool fading)
{
    for (Plot* plt: plots) {
        QCPItemStraightLine* line = new QCPItemStraightLine(plt->plt);
        plt->plt->addItem(line);

        float xval = s.pos.attr(plt->attr_idx); // TODO: handle different parameter layout of OptRerenderer

        // draw vertical line
        line->point1->setCoords(xval, 0);
        line->point2->setCoords(xval, 2);

        // make sure xaxis contains the point
        if (plt->plt->xAxis->range().lower > xval) {
            plt->plt->xAxis->setRangeLower(xval);
        } else if (plt->plt->xAxis->range().upper < xval) {
            plt->plt->xAxis->setRangeUpper(xval);
        }

        // brighten other, older lines
        if (fading) {
            for (size_t i = 0; i < plt->plt->itemCount(); i++) {
                QCPItemStraightLine* item = dynamic_cast<QCPItemStraightLine*>(plt->plt->item(i));
                if (item) {
                    QPen pen = item->pen();
                    int r = pen.color().red();

                    r += 10;
                    if (r > 230) {
                        r = 230;
                    }

                    pen.setColor(QColor(r,r,r));
                    item->setPen(pen);
                }
            }
        }

        plt->plt->replot();
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::clear_points()
{
    for (Plot* plt: plots) {
        plt->plt->clearItems();
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_mouse_move(Plot& plt, QMouseEvent* event)
{
    Statef state = optim->getParams().initial_state;
    double xpos = plt.plt->xAxis->pixelToCoord(event->pos().x());
    double ypos = plt.plt->yAxis->pixelToCoord(event->pos().y());

    state.attr(plt.attr_idx) = xpos;
    emit stateHovered(state);

    ui_cursor_x->setText("X: " + QString::number(xpos));
    ui_cursor_y->setText("Y: " + QString::number(ypos));
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::show_specific_pixel(float x, float y)
{
    for (Plot* plt: plots) {
        plot_single_px(*plt, (int)x, (int)y);
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_total_err_for_all()
{
    for (Plot* plt: plots) {
        plot_total_err(*plt);
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::scale_to_err_only()
{
    for (Plot* plt: plots) {
        plt->plt->graph(0)->rescaleAxes();
        plt->plt->replot();
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::scale_to_jacobi_only()
{
    for (Plot* plt: plots) {
        plt->plt->graph(1)->rescaleAxes();
        plt->plt->graph(3)->rescaleAxes(true);
        plt->plt->replot();
    }
}

////////////////////////////////////////////////////////////////////////////////

void GeneralJacobiTester::plot_numeric_diff(Plot& plt, const std::vector<double>& vals)
{
    QVector<double> xvals(plt.samples-1), yvals(plt.samples-1);

    for (int i = 0; i < plt.samples-1; i++) {

        double x1 = plt.data[i  ].xval;
        double x2 = plt.data[i+1].xval;

        xvals[i] = (x1 + x2)/2;
        yvals[i] = (vals[i+1]-vals[i])/(x2-x1);
    }

    plt.plt->graph(1)->setData(xvals, yvals);
    plt.plt->graph(1)->rescaleAxes(true);
}

////////////////////////////////////////////////////////////////////////////////

