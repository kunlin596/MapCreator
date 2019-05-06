//
// Created by LinKun on 10/20/15.
//

#ifndef MAPCREATOR_COMPUTATIONCONFIGUREDIALOG_H
#define MAPCREATOR_COMPUTATIONCONFIGUREDIALOG_H

#include <QDialog>

#include "PcaKeyFrame_FrameTrackingMethodDialog.h"
#include "FixedFrameCount_FrameTrackingMethodDialog.h"
#include "OneByOne_FrameTrackingMethodDialog.h"

namespace Ui { class ComputationConfigureDialog; }

namespace MapCreator {

	class ComputationConfigureDialog : public QDialog
	{

	Q_OBJECT

	public:

		ComputationConfigureDialog ( QWidget * parent = 0 );

		inline TrackingType GetTrackingType ( ) const { return params_.GetType ( ); }

		Parameters GetParameters ( ) const { return params_; }

		inline bool GetUseGlobalOptimization ( ) const { return use_bundle_adjustment_; }

	private slots:

		void onResultButtonBoxClicked ( QAbstractButton * button );

		void onSettingButtonClicked ( );

	private:

		Ui::ComputationConfigureDialog* ui_;

		bool use_bundle_adjustment_;
		bool options_configured_;

		Parameters params_;

	};
}


#endif //MAPCREATOR_COMPUTATIONCONFIGUREDIALOG_H
