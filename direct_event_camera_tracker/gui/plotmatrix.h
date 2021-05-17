#ifndef PLOTMATRIX_H_8ZHVEYK6
#define PLOTMATRIX_H_8ZHVEYK6

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QPushButton>

#include "point.h"
#include "datasource.h"
#include "optimization/optimization.h"
#include "gui/errorplot.h"
#include "keyframe.h"

class MainGUI;

class PlotMatrix : public QDialog
{
    Q_OBJECT

public:
    PlotMatrix(MainGUI& maingui);

public slots:
    void plot();

    void add_point(const PlotPoint& s, bool fading = false);
    void clear_points();

    virtual void foreach_errplot(boost::function<void(ErrplotLabel*)> f);

signals:
    void stateSelected(const Statef& s);
    void stateHovered (const Statef& s);

protected:
    MainGUI& maingui;
    QGridLayout* layout;
};

#endif /* end of include guard: PLOTMATRIX_H_8ZHVEYK6 */
