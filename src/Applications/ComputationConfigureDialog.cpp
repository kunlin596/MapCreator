//
// Created by LinKun on 10/20/15.
//

#include "ui_ComputationConfigureDialog.h"
#include "Engine/ComputationConfigureDialog.h"

namespace MapCreator {

    ComputationConfigureDialog::ComputationConfigureDialog(QWidget *parent) :
            options_configured_(false),
            ui_(new Ui::ComputationConfigureDialog) {

        ui_->setupUi(this);

        connect(ui_->PushButton_Settings, SIGNAL(clicked()), this,
                SLOT(onSettingButtonClicked()));
        connect(ui_->ButtonBox_ResultButtons, SIGNAL(clicked(QAbstractButton * )), this,
                SLOT(onResultButtonBoxClicked(QAbstractButton * )));;

    }

    void ComputationConfigureDialog::onResultButtonBoxClicked(QAbstractButton *button) {

        QPushButton *_button = (QPushButton * )(button);

        if (_button == (ui_->ButtonBox_ResultButtons->button(QDialogButtonBox::Apply)) and options_configured_) {

            // NOTE: the redesigned TrackerParameters dropped the bundle-adjustment
            // flag and the flat paramsXxx sub-structs. Only the tracking `type`
            // is carried here; the per-method fields would need a polymorphic
            // parameter to survive the base-class assignment (follow-up).

            QDialog::accept();
        }

        else if (_button == (ui_->ButtonBox_ResultButtons->button(QDialogButtonBox::Cancel))) {
            QDialog::reject();
        }
    }

    void ComputationConfigureDialog::onSettingButtonClicked() {

        if (ui_->RadioButton_FrameTrackingMethod_PcaKeyFrameTracking->isChecked()) {

            PcaKeyFrame_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {

                params_ = dialog.GetParameters();
                params_.type = TrackingType::KeyFrameOnly;
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText("KeyFrameOnly tracking configured");
                return;
            }

        } else if (ui_->RadioButton_FrameTrackingMethod_FixedFrameCount->isChecked()) {

            FixedFrameCount_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                params_ = dialog.GetParameters();
                params_.type = TrackingType::FixedNumber;
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText("FixedNumber tracking configured");
                return;
            }

        }
        else if (ui_->RadioButton_FrameTrackingMethod_OneByOne->isChecked()) {

            OneByOne_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                params_ = dialog.GetParameters();
                params_.type = TrackingType::Consecutive;
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText("Consecutive tracking configured");
                return;
            }
        }

        options_configured_ = false;
    }

}
