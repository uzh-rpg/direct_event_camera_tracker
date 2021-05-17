#ifndef IMAGELABEL_H_WXWMH4RP
#define IMAGELABEL_H_WXWMH4RP

#include <QLabel>
#include <QMenu>
#include <QContextMenuEvent>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QPainter>
#include <QImage>

#include <opencv2/opencv.hpp>

#include "utils.h"

// based on https://stackoverflow.com/questions/8211982/qt-resizing-a-qlabel-containing-a-qpixmap-while-keeping-its-aspect-ratio
class ImageLabel : public QLabel
{
    Q_OBJECT
public:
    ImageLabel(QWidget* parent = 0);

    virtual int heightForWidth(int width);
    virtual bool hasHeightForWidth() const { return true; }
    virtual QSize sizeHint() const { return orig_img.size(); }
    virtual QSize minimumSizeHint() const { return orig_img.size(); }
    QPixmap scaledImage() const;

    void setPixmap(const QPixmap& p);
    void setPixmap(cv::Mat mat, bool apply_colormap = false, bool normalize = true);

    virtual void contextMenuEvent(QContextMenuEvent* e);

    void drawAngularBorder(const unsigned int thickness);

    void saveImage(QString filename);

signals:
    void pixelHovered(float x, float y);

public slots:
    void resizeEvent(QResizeEvent *e);
    void saveImageDialog() { saveImage(""); }

protected:
    virtual void mouseMoveEvent(QMouseEvent* e);

    unsigned int border_thickness;
    QPixmap orig_img;
    QPixmap border_img;

    QAction action_save;

    bool scale_smoothly;
};


#endif /* end of include guard: IMAGELABEL_H_WXWMH4RP */
