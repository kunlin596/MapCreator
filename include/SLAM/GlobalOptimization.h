//
// Created by LinKun on 10/29/15.
//

#ifndef MAPCREATOR_GLOBALOPTIMAZATION_H
#define MAPCREATOR_GLOBALOPTIMAZATION_H


#include <opencv2/opencv.hpp>

#include "SLAM/CommonDefinitions.h"
#include "SLAM/CoordinateConverter.h"

#include <eigen3/Eigen/Dense>

namespace MapCreator {


	class GlobalOptimization
	{
	public:

		enum class Type
		{
			LevenbergMarquardt ,
		};

	};

	class LevenbergMarquardt
	{
	public:

		static cv::Matx44f Compute ( const cv::Matx44f & m_before ,
		                             const CoordinateConverter & coordinate_converter ,
		                             const Points & world_points1 ,
		                             const Points & world_points2 );

		static float GetError ( const cv::Matx44f & m ,
		                        const CoordinateConverter & coordinate_converter ,
		                        const Points & world_points1 ,
		                        const Points & world_points2 );

		static float GetError ( const cv::Matx44f & m ,
		                        const CoordinateConverter & coordinate_converter ,
		                        const cv::Point3f & world_point1 ,
		                        const cv::Point3f & world_point2 );

		static cv::Matx44f ToCvMat ( const Eigen::VectorXf & b );

	private:

		// Generic functor
		template < typename _Scalar , int NX = Eigen::Dynamic , int NY = Eigen::Dynamic >
		struct Functor
		{
			using Scalar = _Scalar;
			enum
			{
				InputsAtCompileTime = NX ,
				ValuesAtCompileTime = NY
			};
			using InputType    = Eigen::Matrix < Scalar , InputsAtCompileTime , 1 >;
			using ValueType    = Eigen::Matrix < Scalar , ValuesAtCompileTime , 1 >;
			using JacobianType = Eigen::Matrix < Scalar , ValuesAtCompileTime , InputsAtCompileTime >;
		};

		class NumericalDiffFunctor : public Functor < float >
		{
		public:

			NumericalDiffFunctor ( int inputs , int values ,
			                       const CoordinateConverter & coordinate_converter ,
			                       const Points & world_points1 ,
			                       const Points & world_points2 );

			int operator () ( const Eigen::VectorXf & b , Eigen::VectorXf & fvec ) const;
			int inputs ( ) const { return inputs_; }
			int values ( ) const { return values_; }

		private:
			const int                 inputs_;
			const int                 values_;
			const CoordinateConverter coordinate_converter_;
			const Points              world_points1_;
			const Points              world_points2_;
		};

	};

}


#endif //MAPCREATOR_GLOBALOPTIMAZATION_H
