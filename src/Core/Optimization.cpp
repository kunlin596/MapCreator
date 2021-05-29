//
// Created by LinKun on 10/29/15.
//

#include "Optimization.h"

#include <Core/MyMath.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>


namespace MapCreator {

	cv::Matx44f LevenbergMarquardt::Compute ( const cv::Matx44f & m ,
	                                          const CoordinateConverter & coordinate_converter ,
	                                          const Points & world_points1 ,
	                                          const Points & world_points2 ) {

		// 回転軸
		const auto axis = cv::normalize ( cv::Vec3f ( m ( 1 , 2 ) - m ( 2 , 1 ) , m ( 2 , 0 ) - m ( 0 , 2 ) , m ( 0 , 1 ) - m ( 1 , 0 ) ) );

		// 回転角（ラジアン）
		const auto theta = static_cast<float>(acos ( ( cv::trace ( m ) - 2 ) / 2 ));

		// 並進
		const cv::Vec3f t ( m ( 3 , 0 ) , m ( 3 , 1 ) , m ( 3 , 2 ) );

		const auto n = 7;

		Eigen::VectorXf parameters ( n );

		parameters << axis ( 0 ) , axis ( 1 ) , axis ( 2 ) , theta , t ( 0 ) , t ( 1 ) , t ( 2 );


		NumericalDiffFunctor functor ( n ,
		                               world_points1.size ( ) ,
		                               coordinate_converter ,
		                               world_points1 ,
		                               world_points2 );

        Eigen::NumericalDiff < NumericalDiffFunctor > diff ( functor );

// FIXME
//        Eigen::LevenbergMarquardt < Eigen::NumericalDiff < NumericalDiffFunctor > , NumericalDiffFunctor::Scalar > lm ( diff );

//		const auto status = lm.minimize ( parameters );

		return ToCvMat ( parameters );
	}

	float LevenbergMarquardt::GetError ( const cv::Matx44f & m ,
	                                     const CoordinateConverter & coordinate_converter ,
	                                     const Points & world_points1 ,
	                                     const Points & world_points2 ) {

		boost::accumulators::accumulator_set < float , boost::accumulators::stats < boost::accumulators::tag::mean>> acc;

		for ( size_t i = 0 ; i < world_points1.size ( ) ; ++i ) {

			const auto & p1 = world_points1[ i ];
			const auto & p2 = world_points2[ i ];

			acc ( GetError (
					m ,
					coordinate_converter ,
					p1 ,
					p2 ) );
		}

		return boost::accumulators::extract::mean ( acc );
	}

	float LevenbergMarquardt::GetError ( const cv::Matx44f & m ,
	                                     const CoordinateConverter & coordinate_converter ,
	                                     const cv::Point3f & world_point1 ,         // kp1 : screen
	                                     const cv::Point3f & world_point2 )         // kp2 : world ->screen
	{

		const auto p                    = cv::Vec4f ( world_point2.x , world_point2.y , world_point2.z , 1.0f ) * m;
		const auto aligned_world_point2 = WorldPoint ( p ( 0 ) , p ( 1 ) , p ( 2 ) );

		const auto screen_point1             = coordinate_converter.WorldToScreen ( world_point1 );
		const auto reprojected_screen_point2 = coordinate_converter.WorldToScreen ( aligned_world_point2 );

		const auto diff_x = reprojected_screen_point2.x - screen_point1.x;
		const auto diff_y = reprojected_screen_point2.y - screen_point1.y;
		const auto error  = sqrt ( diff_x * diff_x + diff_y * diff_y );

		return static_cast < float > ( error );
	}

	cv::Matx44f LevenbergMarquardt::ToCvMat ( const Eigen::VectorXf & b ) {

		const cv::Vec3f axis ( b[ 0 ] , b[ 1 ] , b[ 2 ] );      // 回転軸

		const float theta = b[ 3 ];                             // 回転角

		const cv::Vec3f t ( b[ 4 ] , b[ 5 ] , b[ 6 ] );         // 並進

		const cv::Matx33f r = CreateRotationMatrix ( axis , theta );

		cv::Matx44f m ( cv::Matx44f::eye ( ) );

		for ( int i = 0 ; i < 3 ; ++i ) {
			for ( int j = 0 ; j < 3 ; ++j ) {
				m ( i , j ) = r ( i , j );
			}
		}

		m ( 3 , 0 ) = t ( 0 );
		m ( 3 , 1 ) = t ( 1 );
		m ( 3 , 2 ) = t ( 2 );

		return m;
	}


	LevenbergMarquardt::NumericalDiffFunctor::NumericalDiffFunctor ( int inputs , int values ,
	                                                                 const CoordinateConverter & coordinate_converter ,
	                                                                 const Points & world_points1 ,
	                                                                 const Points & world_points2 ) :
			inputs_ ( inputs ) ,
			values_ ( values ) ,
			coordinate_converter_ ( coordinate_converter ) ,
			world_points1_ ( world_points1 ) ,
			world_points2_ ( world_points2 ) { }

	int LevenbergMarquardt::NumericalDiffFunctor::operator () ( const Eigen::VectorXf & b , Eigen::VectorXf & fvec ) const {

		const auto m = ToCvMat ( b );

		for ( int i = 0 ; i < values_ ; ++i ) {

			const auto & p1 = world_points1_[ i ];
			const auto & p2 = world_points2_[ i ];

			fvec[ i ] = LevenbergMarquardt::GetError ( m ,
			                                           coordinate_converter_ ,
			                                           p1 ,
			                                           p2 );
		}

		return 0;

	}

}
