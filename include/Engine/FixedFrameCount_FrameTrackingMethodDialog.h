//
// Created by LinKun on 10/20/15.
//

#ifndef MAPCREATOR_FIXEDFRAMECOUNT_FRAMETRACKINGMETHODDIALOG_H
#define MAPCREATOR_FIXEDFRAMECOUNT_FRAMETRACKINGMETHODDIALOG_H

#include <QDialog>

#include "ui_FixedFrameCount_FrameTrackingMethodDialog.h"

#include <SLAM/Option.h>

namespace Ui {

    class FixedFrameCount_FrameTrackingMethodDialog;
}

namespace MapCreator {

    class FixedFrameCount_FrameTrackingMethodDialog : public QDialog {

        Q_OBJECT

    public:

        FixedFrameCount_FrameTrackingMethodDialog(QWidget *parent = 0);

        inline Options::Options_FixedFrameCount GetOptions() const { return options_; }

    private
        slots:

        void
        onResultButtonBoxClicked ( QAbstractButton
        * button );

    private:

        bool IsValidInput();

        void AcceptDialog();

        void RejectDialog();

        void RestoreDefaultSettings();

        Ui::FixedFrameCount_FrameTrackingMethodDialog ui_;

        Options::Options_FixedFrameCount options_;

    };
}


#endif //MAPCREATOR_FIXEDFRAMECOUNT_FRAMETRACKINGMETHODDIALOG_H
