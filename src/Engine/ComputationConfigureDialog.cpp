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

            params_.UseBundleAdjustment(ui_->CheckBox_UseBundleAdjustment->isChecked());

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

                params_.SetOptionsType(TrackingType::KeyFrameOnly);
                params_.paramsKeyFramesOnly = dialog.GetParameters();
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText(params_.paramsKeyFramesOnly.Output());
                return;
            }

        } else if (ui_->RadioButton_FrameTrackingMethod_FixedFrameCount->isChecked()) {

            FixedFrameCount_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                params_.SetOptionsType(TrackingType::FixedNumber);
                params_.paramsFixedNumber = dialog.GetParameters();
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText(params_.paramsFixedNumber.Output());
                return;
            }

        }
        else if (ui_->RadioButton_FrameTrackingMethod_OneByOne->isChecked()) {

            OneByOne_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                params_.SetOptionsType(TrackingType::Consecutive);
                params_.paramsConsectutive = dialog.GetParameters();
                options_configured_ = true;
                ui_->PlainTextEdit_OptionsSummary->clear();
                ui_->PlainTextEdit_OptionsSummary->appendPlainText(params_.paramsConsectutive.Output());
                return;
            }
        }

        options_configured_ = false;
    }

}