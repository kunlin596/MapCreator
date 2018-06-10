//
// Created by LinKun on 10/20/15.
//

#ifndef NIS_COMPUTATIONCONFIGUREDIALOG_H
#define NIS_COMPUTATIONCONFIGUREDIALOG_H

#include <QDialog>

#include "PcaKeyFrame_FrameTrackingMethodDialog.h"
#include "FixedFrameCount_FrameTrackingMethodDialog.h"
#include "OneByOne_FrameTrackingMethodDialog.h"

#include "../../../bin/lib/MapCreator/ui_ComputationConfigureDialog.h"

namespace Ui { class ComputationConfigureDialog; }

namespace NiS {

	class ComputationConfigureDialog : public QDialog
	{

	Q_OBJECT

	public:

		ComputationConfigureDialog ( QWidget * parent = 0 );

		inline TrackingType GetTrackingType ( ) const { return options_.GetType ( ); }

		Options GetOptions ( ) const { return options_; }

		inline bool GetUseGlobalOptimization ( ) const { return use_bundle_adjustment_; }

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button );

		void onSettingButtonClicked ( );

	private:

		Ui::ComputationConfigureDialog ui_;

		bool use_bundle_adjustment_;
		bool options_configured_;

		Options options_;

	};
}


#endif //NIS_COMPUTATIONCONFIGUREDIALOG_H
