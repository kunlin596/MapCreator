//
// Created by LinKun on 11/4/15.
//

#include "SLAM/CoordinateConverter.h"


namespace NiS {

	const float CoordinateConverter::XtionFrameProperty::kXtionHorizontalFOV = 1.0225999f;
	const float CoordinateConverter::XtionFrameProperty::kXtionVerticalFOV   = 0.79661566f;
	const float CoordinateConverter::XtionFrameProperty::kXtionWidth         = 640;
	const float CoordinateConverter::XtionFrameProperty::kXtionHeight        = 480;

	WorldPoint XtionCoordinateConverter::ScreenToWorld ( ScreenPoint const & screen_point , ushort const depth ) const {

		const auto gz = 0.001f * static_cast<float>(depth);

		WorldPoint p;

		p.x = gz * universal_xz_factor_ * ( screen_point.x / XtionFrameProperty::kXtionWidth - 0.5f );
		p.y = -gz * universal_yz_factor_ * ( screen_point.y / XtionFrameProperty::kXtionHeight - 0.5f );
		p.z = -gz;

		return p;
	}

	ScreenPoint XtionCoordinateConverter::WorldToScreen ( WorldPoint const & world_point ) const {

		const auto x = ( world_point.x / ( world_point.z * universal_xz_factor_ ) + 0.5f ) * XtionFrameProperty::kXtionWidth;
		const auto y = ( world_point.y / ( world_point.z * universal_yz_factor_ ) + 0.5f ) * XtionFrameProperty::kXtionHeight;

		return ScreenPoint ( x , y );
	}

	WorldPoint AistCoordinateConverter::ScreenToWorld ( ScreenPoint const & screen_point , ushort const depth ) const {

		WorldPoint pt;

		if ( depth > 0 ) {

			const float gz = CorrectDepth (
					CorrectDistortion ( static_cast<int>(screen_point.y) , static_cast<int>(screen_point.x) , depth ) ) *
			                 0.001f;

			const float xz_factor = NthDegreeEquation ( internal_calibration_info_.hfov_calibration_vector , gz );
			const float yz_factor = NthDegreeEquation ( internal_calibration_info_.vfov_calibration_vector , gz );
			const float gx        = xz_factor * ( static_cast< float >( screen_point.x ) / XtionFrameProperty::kXtionWidth - 0.5f );
			const float gy        = yz_factor * ( static_cast< float >( screen_point.y ) / XtionFrameProperty::kXtionHeight - 0.5f );

			pt.x = gx;
			pt.y = -gy;
			pt.z = -gz;
		}

		return pt;
	}

	ScreenPoint AistCoordinateConverter::WorldToScreen ( WorldPoint const & world_point ) const {

		const float xz_factor = NthDegreeEquation ( internal_calibration_info_.hfov_calibration_vector , world_point.z );
		const float yz_factor = NthDegreeEquation ( internal_calibration_info_.vfov_calibration_vector , world_point.z );
		const float x         = ( world_point.x / xz_factor + 0.5f ) * XtionFrameProperty::kXtionWidth;
		const float y         = ( world_point.y / yz_factor + 0.5f ) * XtionFrameProperty::kXtionHeight;
		return ScreenPoint ( x , y );
	}


}