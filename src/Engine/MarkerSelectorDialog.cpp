//
// Created by LinKun on 10/22/15.
//

#include "MapCreator/MarkerSelectorDialog.h"

namespace MapCreator {

    MarkerSelectorDialog::MarkerSelectorDialog(QWidget *parent) {

        ui_.setupUi(this);
        setMouseTracking(true);
    }

    void MarkerSelectorDialog::SetKeyFrame(const KeyFrame &keyframe) {

        keyframe_ = keyframe;

        ColorImage color = keyframe.GetColorImage();

        QImage display_image(color.data, color.cols, color.rows, QImage::Format::Format_RGB888);

        ui_.Label_Image->setPixmap(QPixmap::fromImage(display_image));

        ui_.Label_Image->setFixedWidth(display_image.width());
        ui_.Label_Image->setFixedHeight(display_image.height());

        setFixedSize(sizeHint());
    }

    void MarkerSelectorDialog::mousePressEvent(QMouseEvent *e) {

        if (ui_.Label_Image->rect().contains(e->pos())) {
            const cv::Vec3f point = keyframe_.GetPointImage().at<cv::Vec3f>(e->y(), e->x());
            ui_.LineEdit_ResultPointX->setText(QString("%1").arg(point(0)));
            ui_.LineEdit_ResultPointY->setText(QString("%1").arg(point(1)));
            ui_.LineEdit_ResultPointZ->setText(QString("%1").arg(point(2)));
        }
    }

    void MarkerSelectorDialog::mouseMoveEvent(QMouseEvent *e) {

        if (ui_.Label_Image->geometry().contains(e->pos())) {
            const cv::Vec3f point = keyframe_.GetPointImage().at<cv::Vec3f>(e->y(), e->x());
            ui_.LineEdit_MousePositionX->setText(QString("%1").arg(point(0)));
            ui_.LineEdit_MousePositionY->setText(QString("%1").arg(point(1)));
            ui_.LineEdit_MousePositionZ->setText(QString("%1").arg(point(2)));
        }
    }

    void MarkerSelectorDialog::mouseDoubleClickEvent(QMouseEvent *e) {

        if (ui_.Label_Image->geometry().contains(e->pos())) {
            const cv::Vec3f point = keyframe_.GetPointImage().at<cv::Vec3f>(e->y(), e->x());
            ui_.LineEdit_MousePositionX->setText(QString("%1").arg(point(0)));
            ui_.LineEdit_MousePositionY->setText(QString("%1").arg(point(1)));
            ui_.LineEdit_MousePositionZ->setText(QString("%1").arg(point(2)));

            point_.x = point(0);
            point_.y = point(1);
            point_.z = point(2);

            hide();

            emit FetchPointDone();
        }
    }

}