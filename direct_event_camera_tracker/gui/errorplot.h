#ifndef ERRORPLOT_H_6Y57S3M2
#define ERRORPLOT_H_6Y57S3M2

#include <list>

#include <QLabel>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QColor>
#include <QRectF>
#include <QPointF>
#include <QProgressBar>
#include <QCoreApplication>
#include <QProgressDialog>
#include <QElapsedTimer>

#include <opencv2/opencv.hpp>

#include "point.h"
#include "event_buffer.h"
#include "gui/ImageLabel.h"
#include "keyframe.h"
#include "optimization/optimization.h"

////////////////////////////////////////////////////////////////////////////////

struct PlotPoint
{
    Statef pos;
    QColor color;

    enum Shape {
        CIRCLE,
        FILLED_CIRCLE,
        CROSS,
        PLUS,
        POINT,
        SQUARE,
        FILLED_SQUARE,
    } shape;

    PlotPoint(const Statef& pos, const Shape& shape = CIRCLE, const QColor& color = Qt::white)
        : pos(pos), color(color), shape(shape) {};

    void draw(QPainter& painter, Eigen::Vector2d center) const;
};

////////////////////////////////////////////////////////////////////////////////

struct PlotRange
{
    PlotRange()
        : dimX(0), dimY(0), fromX(0), toX(0), fromY(0), toY(0), samples(0) {};

    PlotRange(Statef center, size_t dimX, size_t dimY, double fromX, double toX, double fromY, double toY, size_t samples)
        : center(center), dimX(dimX), dimY(dimY), fromX(fromX), toX(toX), fromY(fromY), toY(toY), samples(samples) {};

    PlotRange(Statef center, size_t dimX, size_t dimY, double divX, double divY, size_t samples)
        : center(center), dimX(dimX), dimY(dimY), fromX(-divX), toX(divX), fromY(-divY), toY(divY), samples(samples) {};

    Statef center;
    size_t dimX, dimY;
    double fromX, toX, fromY, toY;
    size_t samples;

    //! x/y_idx in [0,samples]
    Statef getState(const size_t x_idx, const size_t y_idx) const;
    //! x/y in [0,1] (can also be bigger/smaller, but 0=from, 1=to)
    Statef getState(const double x_idx, const double y_idx) const;

    Eigen::Vector2d getFraction(const Statef& s) const;
};

////////////////////////////////////////////////////////////////////////////////

class ErrplotLabel : public ImageLabel
{
    Q_OBJECT
public:
    ErrplotLabel(QWidget* parent = 0);

    void setPlotRange(const PlotRange& r) { range = r; }

    Statef getStateFromPixel(const QPointF& p);

    void clearPoints();
    void addPoint(const PlotPoint& pt);
    void addPointFading(const PlotPoint& pt);

    virtual void contextMenuEvent(QContextMenuEvent* e);

    //! plots correlation around given pose
    void plot(
            const PlotRange& range,
            IOptimization& optim);

    void saveGUIimage(const QString& filename);

signals:
    void stateSelected(const Statef& s);
    void stateHovered (const Statef& s);

protected:
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void mousePressEvent(QMouseEvent* e);

    virtual void paintEvent(QPaintEvent* e);
    void drawPoint(const PlotPoint& s, QPainter& painter);

private:
    PlotRange range;
    std::list<PlotPoint> points;

    QAction action_clear_pts;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: ERRORPLOT_H_6Y57S3M2 */
