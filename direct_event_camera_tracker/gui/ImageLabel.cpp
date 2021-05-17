#include "ImageLabel.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

ImageLabel::ImageLabel(QWidget* parent)
    : QLabel(parent),
      border_thickness(0),
      orig_img(10,10),
      action_save(tr("&Save Image"), this),
      scale_smoothly(true)
{
    setScaledContents(false);
    QSizePolicy policy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);
    setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

    connect(&action_save, &QAction::triggered, this, &ImageLabel::saveImageDialog);

    orig_img.fill(Qt::red);
    QLabel::setPixmap(scaledImage());
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::setPixmap(const QPixmap& p)
{
    if (orig_img.size() != p.size()) {
        // force redrawing of border if image size is changing
        border_img = QPixmap();
    }

    orig_img = p;

    if (border_thickness > 0) {
        drawAngularBorder(border_thickness);
        QPainter painter(&border_img);
        painter.drawPixmap(border_thickness, border_thickness, p);
    }

    QLabel::setPixmap(scaledImage());
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::setPixmap(cv::Mat mat, bool apply_colormap, bool normalize)
{
    if (mat.type() == CV_64FC1) {

        // convert grayscale float image into colorful 8bit RGB picture
        if (apply_colormap) {
            mat.convertTo(mat, CV_8UC1, 255);
            cv::applyColorMap(mat, mat, cv::COLORMAP_JET);
        } else if (normalize) {
            cv::normalize(mat, mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        } else {
            mat.convertTo(mat, CV_8UC1, 255);
        }
    } else if (mat.type() == CV_64FC2) {
        visualize_gradient(mat, mat);
        mat.convertTo(mat, CV_8UC3, 255);

    } else if (mat.type() == CV_64FC3) {
        mat.convertTo(mat, CV_8UC3, 255);
    }


    if (mat.type() == CV_8UC1) {
        setPixmap(QPixmap::fromImage(QImage(
                        mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8)));

    } else if (mat.type() == CV_8UC3) {

        // Qt expects RGB, but OpenCV uses BGR
        cv::cvtColor(mat, mat, CV_BGR2RGB);

        setPixmap(QPixmap::fromImage(QImage(
                        mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888)));
    } else {
        cerr << "ERROR: unknown cv image type " << mat.type() << endl;
        return;
    }


    if (orig_img.width() == 0) {
        cerr << "ERROR: image conversion failed! type: " << mat.type() << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

int ImageLabel::heightForWidth(int width)
{
    if (!orig_img.isNull()) {
        if (width >= orig_img.width()) {
            return orig_img.height();
        } else {
            return ((qreal)orig_img.height()*width) / orig_img.width();
        }
    } else {
        return height();
    }
}

////////////////////////////////////////////////////////////////////////////////

QPixmap ImageLabel::scaledImage() const
{
    if (orig_img.isNull()) {
        return orig_img;
    }

    const QPixmap& pix = (border_thickness > 0 ? border_img : orig_img );
    return pix.scaled(size(), Qt::KeepAspectRatio, scale_smoothly?Qt::SmoothTransformation:Qt::FastTransformation);
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::resizeEvent(QResizeEvent *)
{
    if (!orig_img.isNull()) {
        QLabel::setPixmap(scaledImage());
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::contextMenuEvent(QContextMenuEvent* e)
{
    QMenu menu(this);
    menu.addAction(&action_save);
    menu.exec(e->globalPos());
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::saveImage(QString filename)
{
    if (filename.isEmpty()) {
        filename = QFileDialog::getSaveFileName(this,
                tr("Save Image"),
                "~/Pictures/",
                tr("Images (*.png; *.jpg)")
        );
    }

    if (!filename.isEmpty()) {
        if (!filename.contains('.')) {
            filename += ".png";
        }

        // create folder if it doesn't exist yet
        QDir dir = QFileInfo(filename).absoluteDir();
        if (!dir.mkpath(".")) {
            cerr << "Failed to create folder: '" << dir.path().toStdString() << "'" << endl;
        }

        // TODO: append extension
        if (!orig_img.save(filename)) {
            QMessageBox::warning(this, "Error saving image", "Could not save image to '" + filename + "'.");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::drawAngularBorder(const unsigned int thickness)
{
    // don't redraw if we don't have to
    if (border_thickness == thickness && !border_img.isNull()) {
        return;
    }

    border_thickness = thickness;

    if (orig_img.isNull()) {
        //cerr << "Cannot draw border: No image set." << endl;
        return;
    }

    QImage img = QImage(orig_img.size() + QSize(thickness*2, thickness*2), QImage::Format_RGB888);
    img.fill(Qt::black);

    /*
    QPainter painter(&border_img);
    painter.setPen(Qt::red);
    painter.drawRect(0, 0, border_img.width()-1, border_img.height()-1);
    */

    auto calc_color = [&](const double x, const double y) -> QRgb {
        double angle = cv::fastAtan2(y-img.height()/2, x-img.width()/2);
        return QColor::fromHsv(angle, 255, 255).rgb();
    };

    for (unsigned int offset = 0; offset < thickness-1; offset++) {
        // horizontal
        for (unsigned int x = offset; x < img.width()-offset; x++) {
            img.setPixel(x, offset, calc_color(x,offset));
            img.setPixel(x, img.height()-offset-1, calc_color(x,img.height()-offset-1));
        }

        // vertical
        for (unsigned int y = offset; y < img.height()-offset; y++) {
            img.setPixel(offset, y, calc_color(offset,y));
            img.setPixel(img.width()-offset-1, y, calc_color(img.width()-offset-1,y));
        }
    }

    border_img = QPixmap::fromImage(img);
}

////////////////////////////////////////////////////////////////////////////////

void ImageLabel::mouseMoveEvent(QMouseEvent* e)
{
    QPointF p = e->localPos();

    double frac_x = 0, frac_y = 0;
    if (width() > pixmap()->width()) {
        // image has border at left & right (i.e. it is centered horizontally and touching top & bottom)
        frac_x = (p.x() - (width()-pixmap()->width())/2) / ((double)pixmap()->width());
        frac_y = p.y() / ((double)height());
    } else {
        frac_x = p.x() / ((double)width());
        frac_y = (p.y() - (height()-pixmap()->height())/2) / ((double)pixmap()->height());
    }

    if (frac_x >= 0 && frac_x < 1
    &&  frac_y >= 0 && frac_y < 1) {
        double x = frac_x * orig_img.width();
        double y = frac_y * orig_img.height();
        emit pixelHovered(x, y);
    }
}

////////////////////////////////////////////////////////////////////////////////

