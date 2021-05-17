#include "plotmatrix.h"
#include "gui/maingui.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

PlotMatrix::PlotMatrix(MainGUI& maingui)
    : QDialog(&maingui), maingui(maingui), layout(new QGridLayout(this))
{
    setWindowTitle("Plot Matrix");

    QStringList hdrs = Statef::get_attr_names();
    for (size_t r = 0; r < hdrs.length(); r++) {

        // range inputs
        QDoubleSpinBox* range = new QDoubleSpinBox(this);
        range->setMaximum(100);
        range->setMinimum(-100);
        range->setDecimals(5);
        range->setSingleStep(0.01);
        range->setValue(r < 7 ? 0.03 : 3);
        layout->addWidget(range, 1, r);

        if (r == hdrs.length()-1) {
            break;
        }

        // plot images
        for (size_t c = r+1; c < hdrs.length(); c++) {
            ErrplotLabel* plt = new ErrplotLabel(this);
            layout->addWidget(plt, r+2, c);

            connect(plt, &ErrplotLabel::stateHovered,  this, &PlotMatrix::stateHovered);
            connect(plt, &ErrplotLabel::stateSelected, this, &PlotMatrix::stateSelected);
        }

        // horizontal
        layout->addWidget(new QLabel(hdrs[r+1]), 0, r+1);

        // vertical
        QLabel* lbl = new QLabel(hdrs[r]);
        // TODO: implement a QVerticalLabel class...
        layout->addWidget(lbl, r+2, 0);
    }


    QPushButton* plot_button = new QPushButton("plot", this);
    layout->addWidget(plot_button, 0, 0);
    connect(plot_button, &QPushButton::clicked, this, &PlotMatrix::plot);


    QPushButton* clear_pts_button = new QPushButton("clear points", this);
    layout->addWidget(clear_pts_button, 5, 2);
    connect(clear_pts_button, &QPushButton::clicked, this, &PlotMatrix::clear_points);

    setLayout(layout);
}

////////////////////////////////////////////////////////////////////////////////

void PlotMatrix::plot()
{
    clear_points();

    std::unique_ptr<IOptimization> optim = maingui.build_optimization(Statef());

    const size_t N = Statef::get_attr_names().length();
    for (size_t r = 0; r < N-1; r++) {
        for (size_t c = r+1; c < N; c++) {

            cout << "plotting " << Statef::get_attr_names()[c].toStdString() << " vs. " << Statef::get_attr_names()[r].toStdString() << endl;

            ErrplotLabel* plt = (ErrplotLabel*) layout->itemAtPosition(r+2, c)->widget();

            QDoubleSpinBox* dim1_dev = (QDoubleSpinBox*) layout->itemAtPosition(1, c)->widget();
            QDoubleSpinBox* dim2_dev = (QDoubleSpinBox*) layout->itemAtPosition(1, r)->widget();

            PlotRange range(
                    maingui.get_T_WC_selected(),
                    c, r,
                    dim1_dev->value(), dim2_dev->value(),
                    (size_t) maingui.getUI().samples_count->value());

            plt->plot(range, *optim);

            update();
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void PlotMatrix::add_point(const PlotPoint& s, bool fading)
{
    foreach_errplot([s, fading](ErrplotLabel* plt){
            if (fading) {
                plt->addPointFading(s);
            } else {
                plt->addPoint(s);
            }
    });
}

////////////////////////////////////////////////////////////////////////////////

void PlotMatrix::clear_points()
{
    foreach_errplot([](ErrplotLabel* plt){
            plt->clearPoints();
    });
}

////////////////////////////////////////////////////////////////////////////////

void PlotMatrix::foreach_errplot(boost::function<void(ErrplotLabel*)> f)
{
    const size_t N = Statef::get_attr_names().length();
    for (size_t r = 0; r < N-1; r++) {
        for (size_t c = r+1; c < N; c++) {
            ErrplotLabel* plt = (ErrplotLabel*) layout->itemAtPosition(r+2, c)->widget();
            f(plt);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

