#include "errorplot.h"

using namespace std;
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////

void PlotPoint::draw(QPainter& painter, Eigen::Vector2d center) const
{
    painter.setPen(color);

    switch (shape) {
        case FILLED_CIRCLE:
            painter.setBrush(Qt::SolidPattern);
        case CIRCLE:
            painter.drawEllipse(QPointF(center.x()-3, center.y()-3), 5, 5);
            break;

        case CROSS:
            painter.drawLine(QPointF(center.x()-3, center.y()-3), QPointF(center.x()+3, center.y()+3));
            painter.drawLine(QPointF(center.x()+3, center.y()-3), QPointF(center.x()-3, center.y()+3));
            break;

        case PLUS:
            painter.drawLine(QPointF(center.x()-3, center.y()  ), QPointF(center.x()+3, center.y()  ));
            painter.drawLine(QPointF(center.x()  , center.y()-3), QPointF(center.x()  , center.y()+3));
            break;

        case POINT:
            painter.drawPoint(QPointF(center.x(), center.y()));
            break;

        case FILLED_SQUARE:
            painter.setBrush(Qt::SolidPattern);
        case SQUARE:
            painter.drawRect(QRectF(center.x()-3, center.y()-3, 5, 5));
            break;

        default:
            cerr << "ERROR: shape " << shape << " not implemented in PlotPoint." << endl;
            break;
    }

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Statef PlotRange::getState(const size_t x_idx, const size_t y_idx) const
{
    if (samples > 0) {
        return getState( ((double)x_idx)/samples, ((double)y_idx)/samples );
    } else {
        return Statef();
    }
}

////////////////////////////////////////////////////////////////////////////////

Statef PlotRange::getState(const double x_idx, const double y_idx) const
{
    Statef tmp_state = center;
    tmp_state.attr(dimX) += x_idx * (toX-fromX) + fromX;
    tmp_state.attr(dimY) += y_idx * (toY-fromY) + fromY;
    return tmp_state;
}

////////////////////////////////////////////////////////////////////////////////

Eigen::Vector2d PlotRange::getFraction(const Statef& s) const
{
    Vector2d f;
    f.x() = (s.attr(dimX) - (center.attr(dimX)+fromX)) / (toX - fromX);
    f.y() = (s.attr(dimY) - (center.attr(dimY)+fromY)) / (toY - fromY);
    return f;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ErrplotLabel::ErrplotLabel(QWidget* parent)
    : ImageLabel(parent),
    action_clear_pts(tr("clear points"), this)
{
    // we want to receive mose-over events
    setMouseTracking(true);

    scale_smoothly = false;

    connect(&action_clear_pts, &QAction::triggered, this, &ErrplotLabel::clearPoints);
}

////////////////////////////////////////////////////////////////////////////////

Statef ErrplotLabel::getStateFromPixel(const QPointF& p)
{
    if (orig_img.isNull() || !pixmap()) {
        return Statef();
    }

    double frac_x = 0, frac_y = 0;
    if (width() > pixmap()->width()) {
        // image has border at left & right (i.e. it is centered horizontally and touching top & bottom)
        frac_x = (p.x() - (width()-pixmap()->width())/2) / ((double)pixmap()->width());
        frac_y = p.y() / ((double)height());
    } else {
        frac_x = p.x() / ((double)width());
        frac_y = (p.y() - (height()-pixmap()->height())/2) / ((double)pixmap()->height());
    }

    // frac_x/y is now a double in [0-d,1+d]
    return range.getState(frac_x, frac_y);
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::mouseMoveEvent(QMouseEvent* e)
{
    Statef s = getStateFromPixel(e->localPos());
    if (s.valid) {
        emit stateHovered(s);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::mousePressEvent(QMouseEvent* e)
{
    Statef s = getStateFromPixel(e->localPos());
    if (s.valid) {
        emit stateSelected(s);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::clearPoints()
{
    points.clear();
    update();
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::addPoint(const PlotPoint& pt)
{
    points.push_back(pt);
    update();
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::addPointFading(const PlotPoint& pt)
{
    // fade out all existing points
    for(PlotPoint& p: points) {
        p.color = p.color.darker(110);
    }

    addPoint(pt);
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::contextMenuEvent(QContextMenuEvent* e)
{
    QMenu menu(this);
    menu.addAction(&action_save);
    menu.addAction(&action_clear_pts);
    menu.exec(e->globalPos());
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::paintEvent(QPaintEvent* e)
{
    ImageLabel::paintEvent(e);

    if (orig_img.isNull()) {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    for(const PlotPoint& s: points) {
        drawPoint(s, painter);
    }

    painter.setPen(Qt::white);
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::drawPoint(const PlotPoint& s, QPainter& painter)
{
    Vector2d frac = range.getFraction(s.pos);
    //cout << "Statef maps to " << frac.transpose() << "  -> ";

    Vector2d img_orig(0,0);
    if (width() > pixmap()->width()) {
        // image has border at left & right (i.e. it is centered horizontally and touching top & bottom)
        img_orig.x() = (width()-pixmap()->width())/2.;
    } else {
        img_orig.y() = (height()-pixmap()->height())/2.;
    }

    Vector2d point_pos = img_orig.array() + frac.array() * Vector2d(pixmap()->width(), pixmap()->height()).array();
    //cout << point_pos.transpose() << endl;

    s.draw(painter, point_pos);
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::plot(
        const PlotRange& range,
        IOptimization& optim)
{
    cout << "plotting error with " << range.samples << " samples" << endl;

    //high_resolution_clock::time_point start_t = high_resolution_clock::now();
    QProgressDialog progress("Rendering error plot...", "abort", 0, range.samples*range.samples, this);
    progress.setWindowModality(Qt::WindowModal);

    cv::Mat err(range.samples, range.samples, CV_64FC1, cv::Scalar(0));

    QElapsedTimer timer;
    timer.start();

    for (size_t dy_idx = 0; dy_idx < range.samples; dy_idx++) {

        for (size_t dx_idx = 0; dx_idx < range.samples; dx_idx++) {

            progress.setValue(dy_idx*range.samples + dx_idx);
            if (progress.wasCanceled()) {
                return;
            }

            Statef T_CK = range.getState(dx_idx, dy_idx);

            double e = optim.error(T_CK, {});

            if (isnan(e) || isinf(e)) {
                cerr << "WARNING: Invalid correlation " << dx_idx << " / " << range.samples << " at position: " << e << endl;
                cerr << T_CK;
            }

            err.at<double>(dy_idx,dx_idx) = e;
        }

    }

    long int ns = timer.nsecsElapsed();
    long int N = range.samples*range.samples;
    cout << "rendering " << N << " datapoints took " << ns/1000/1000 << " milliseconds, which equals " << double(ns)/(1000*1000*N) << " ms per point" << endl;

    cv::normalize(err, err, 0, 1, cv::NORM_MINMAX);

    setPixmap(err, true);
    setPlotRange(range);

    progress.close();
}

////////////////////////////////////////////////////////////////////////////////

void ErrplotLabel::saveGUIimage(const QString& filename)
{
    QPixmap target(orig_img.size()*40);
    target.fill();

    this->render(&target);

    if (!target.save(filename)) {
        QMessageBox::warning(this, "Error saving image", "Could not save image to '" + filename + "'.");
    }
}

////////////////////////////////////////////////////////////////////////////////


