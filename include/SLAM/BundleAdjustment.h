//
// Created by LinKun on 11/4/15.
//

#ifndef MAPCREATOR_BUNDDLEADJUSTMENT_H
#define MAPCREATOR_BUNDDLEADJUSTMENT_H

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

namespace MapCreator {

    class BundleAdjustment {
    public:

//		static cv::Matx44f Compute ( cv::Matx44f const & m_before );
//		static float       GetError ( cv::Matx44f const & m );
//

    private:

//		// Generic functor
//		template < typename _Scalar , int NX = Eigen::Dynamic , int NY = Eigen::Dynamic >
//		struct Functor
//		{
//			using Scalar = _Scalar;
//			enum
//			{
//				InputsAtCompileTime = NX ,
//				ValuesAtCompileTime = NY
//			};
//			using InputType    = Eigen::Matrix < Scalar , InputsAtCompileTime , 1 >;
//			using ValueType    = Eigen::Matrix < Scalar , ValuesAtCompileTime , 1 >;
//			using JacobianType = Eigen::Matrix < Scalar , ValuesAtCompileTime , InputsAtCompileTime >;
//		};
//
//		class NumericalDiffFunctor : public Functor < float >
//		{
//		public:
//			NumericalDiffFunctor ( int inputs , int values ,
//			                       const InternalCalibrations & internal_calibrations ,
//			                       const ExternalCalibrations & external_calibrations ,
//			                       const MatchedPoints & inlier_points1 ,
//			                       const MatchedPoints & inlier_points2 );
//
//			int operator () ( const Eigen::VectorXf & b , Eigen::VectorXf & fvec ) const;
//			int inputs ( ) const { return inputs_; }
//			int values ( ) const { return values_; }
//
//		private:
//			const int inputs_;
//			const int values_;
//			const InternalCalibrations & internal_calibrations_;
//			const ExternalCalibrations & external_calibrations_;
//			const MatchedPoints        & inlier_points1_;
//			const MatchedPoints        & inlier_points2_;
//		};
    };
}


#endif //MAPCREATOR_BUNDDLEADJUSTMENT_H
