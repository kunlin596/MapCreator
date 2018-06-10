//
// Created by LinKun on 10/20/15.
//

#include "MapCreator/ComputationConfigureDialog.h"

namespace NiS {

    ComputationConfigureDialog::ComputationConfigureDialog(QWidget *parent) :
            options_configured_(false) {

        ui_.setupUi(this);

        connect(ui_.PushButton_Settings, SIGNAL(clicked()), this,
                SLOT(onSettingButtonClicked()));
        connect(ui_.ButtonBox_ResultButtons, SIGNAL(clicked(QAbstractButton * )), this,
                SLOT(onResultButtonBoxClicked(QAbstractButton * )));;

    }

    void ComputationConfigureDialog::onResultButtonBoxClicked(QAbstractButton *button) {

        QPushButton *_button = (QPushButton * )(button);

        if (_button == (ui_.ButtonBox_ResultButtons->button(QDialogButtonBox::Apply)) and options_configured_) {

            options_.UseBundleAdjustment(ui_.CheckBox_UseBundleAdjustment->isChecked());

            QDialog::accept();
        }

        else if (_button == (ui_.ButtonBox_ResultButtons->button(QDialogButtonBox::Cancel))) {
            QDialog::reject();
        }
    }

    void ComputationConfigureDialog::onSettingButtonClicked() {

        if (ui_.RadioButton_FrameTrackingMethod_PcaKeyFrameTracking->isChecked()) {

            PcaKeyFrame_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {

                options_.SetOptionsType(TrackingType::PcaKeyFrame);
                options_.options_pca_keyframe = dialog.GetOptions();
                options_configured_ = true;
                ui_.PlainTextEdit_OptionsSummary->clear();
                ui_.PlainTextEdit_OptionsSummary->appendPlainText(options_.options_pca_keyframe.Output());
                return;
            }

        } else if (ui_.RadioButton_FrameTrackingMethod_FixedFrameCount->isChecked()) {

            FixedFrameCount_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                options_.SetOptionsType(TrackingType::FixedFrameCount);
                options_.options_fixed_frame_count = dialog.GetOptions();
                options_configured_ = true;
                ui_.PlainTextEdit_OptionsSummary->clear();
                ui_.PlainTextEdit_OptionsSummary->appendPlainText(options_.options_fixed_frame_count.Output());
                return;
            }

        }
        else if (ui_.RadioButton_FrameTrackingMethod_OneByOne->isChecked()) {

            OneByOne_FrameTrackingMethodDialog dialog;
            if (dialog.exec() == QDialog::Accepted) {
                options_.SetOptionsType(TrackingType::OneByOne);
                options_.options_one_by_one = dialog.GetOptions();
                options_configured_ = true;
                ui_.PlainTextEdit_OptionsSummary->clear();
                ui_.PlainTextEdit_OptionsSummary->appendPlainText(options_.options_one_by_one.Output());
                return;
            }
        }

        options_configured_ = false;
    }

}