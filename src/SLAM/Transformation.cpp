//
// Created by LinKun on 9/13/15.
//

#include "SLAM/Transformation.h"
#include "SLAM/CommonDefinitions.h"

#include <random>
#include <fstream>
#include <set>

#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/matrix_decompose.hpp>
// #include <gnuplot-iostream.h>

#include <boost/tuple/tuple.hpp>

namespace {

	using MapCreator::Points;
	using Errors = std::vector < double >;

	template < typename T = float >
	class SolveRot_SVD    // LMS estimation of rotation matrix between point pairs by SVD
	{
	public:
		cv::Mat operator () ( const cv::Mat & cov ) const {

			using namespace cv;
			using namespace std;

			SVD svd ( cov , SVD::MODIFY_A );
			Mat sgn = Mat::diag ( Mat_ < T > ( Vec < T , 3 > ( 1 , 1 , determinant ( svd.u * svd.vt ) ) ) );

			// sgn : CV_32F, svd.u : CV_64F, svd.vt : CV64F
			// need conversion
			Mat cvt_svdu;
			svd.u.convertTo ( cvt_svdu , CV_32F );

			Mat cvt_svdvt;
			svd.vt.convertTo ( cvt_svdvt , CV_32F );

			Mat rot = cvt_svdu * sgn * cvt_svdvt;

			return rot;
		}
	};

	// float version
	struct SolveRotationMatrix_SVD
	{
		cv::Mat operator () ( const cv::Mat & cov ) const {

			using namespace cv;
			SVD svd ( cov , SVD::MODIFY_A );
			Mat sgn = Mat::diag ( Mat_ < float > ( Vec3f ( 1 , 1 , ( float ) determinant ( svd.u * svd.vt ) ) ) );
			Mat rot = svd.u * sgn * svd.vt;
			return rot;
		}
	};


	// PCL の TransformationFromCorrespondences と同じものを OpenCV で実装しました
	template < class T >
	class TransformationFromCorrespondences
	{
	private:

		using Mat = cv::Mat_ < T >;

	public:
		TransformationFromCorrespondences ( ) {

			Reset ( );
		}

		void Reset ( ) {

			samples_            = 0;
			accumulated_weight_ = 0;
			mean1_              = Mat ( 1 , 3 , 0.0 );
			mean2_              = Mat ( 1 , 3 , 0.0 );
			covariance_         = Mat ( 3 , 3 , 0.0 );
		}

		void Add ( const cv::Point3f & point1 , const cv::Point3f & point2 ) {

			++samples_;
			accumulated_weight_ += 1.0;
			const T alpha = 1.0 / accumulated_weight_;

			const Mat diff1 = ( Mat ) ( ToMat ( point1 ) - mean1_ );
			const Mat diff2 = ( Mat ) ( ToMat ( point2 ) - mean2_ );
			covariance_ = ( 1.0 - alpha ) * ( covariance_ + alpha * ( diff1.t ( ) * diff2 ) );

			mean1_ += alpha * diff1;
			mean2_ += alpha * diff2;
		}

		cv::Matx44f GetTransformation ( ) const {

			using namespace std;

			SolveRot_SVD < float > svd;
			const Mat              r = svd ( covariance_ );        // 3x3
			const Mat              t = ( Mat ) ( mean2_ - ( mean1_ * r ) );

			cv::Matx44f m = cv::Matx44f::eye ( );
			m << cv::Matx33f ( r ) << cv::Vec3f ( t ( 0 , 0 ) , t ( 0 , 1 ) , t ( 0 , 2 ) );
			return m;
		}

	private:
		unsigned int samples_;
		T            accumulated_weight_;
		Mat          mean1_;
		Mat          mean2_;
		Mat          covariance_;

		Mat ToMat ( const cv::Point3f & p ) {

			Mat m ( 1 , 3 );
			m ( 0 , 0 ) = p.x;
			m ( 0 , 1 ) = p.y;
			m ( 0 , 2 ) = p.z;
			return m;
		}
	};

	// エラーを計算する
	Errors ComputeErrors ( const Points & points1 , const Points & points2 , const cv::Matx44f & m ) {

		Errors errors ( points1.size ( ) );

		for ( size_t i = 0 ; i < points1.size ( ) ; ++i ) {
			const cv::Point3f & p1 = points1[ i ];
			const cv::Point3f & p2 = points2[ i ];
			const cv::Vec4f v1 ( p1.x , p1.y , p1.z , 1.0f );
			const cv::Vec4f v2 ( p2.x , p2.y , p2.z , 1.0f );

			const cv::Vec4f v = v1 * m - v2;
			errors[ i ] = cv::norm ( v );
		}

		return errors;
	}
	Errors ComputeFlowErrors ( const Points & points1 , const Points & points2 ) {

		Errors errors ( points1.size ( ) );

		cv::Point3f _mean1;
		cv::Point3f _mean2;

		std::for_each ( points1.begin ( ) , points1.end ( ) , [ & ] ( const cv::Point3f & point ) -> void { _mean1 += point; } );
		std::for_each ( points2.begin ( ) , points2.end ( ) , [ & ] ( const cv::Point3f & point ) -> void { _mean2 += point; } );

		_mean1.x /= points1.size ( );
		_mean1.y /= points1.size ( );
		_mean1.z /= points1.size ( );

		_mean2.x /= points2.size ( );
		_mean2.y /= points2.size ( );
		_mean2.z /= points2.size ( );

		auto centroid_diff = _mean1 - _mean2;

		for ( auto i = 0 ; i < errors.size ( ) ; ++i ) {

			const auto & p1 = points1[ i ];
			const auto & p2 = points2[ i ] + centroid_diff;

			glm::vec3 vec1 ( p1.x , p1.y , p1.z );
			glm::vec3 vec2 ( p2.x , p2.y , p2.z );

			float _val1 = glm::dot ( vec1 , vec2 );
			float _val2 = glm::length ( vec1 ) * glm::length ( vec2 );

			auto _angle = acosf ( _val1 / _val2 ) / glm::pi < float > ( ) * 180.0f;

			auto _cross_product = glm::cross ( vec2 , vec1 );
			auto _val3          = glm::dot ( _cross_product , glm::vec3 ( 0.0f , 1.0f , 0.0f ) );

			int sgn = ( _val3 > 0.0f ) ? ( 1 ) : ( -1 );

			// const auto angle = glm::angle ( vec1 , vec2 ); // ERROR???
			errors[ i ] = sgn * _angle;

		}

		return errors;
	}

	// 初期行列を求める RANSAC
	std::pair < int , cv::Matx44f > ComputeInitialMatrix ( const Points & points1 ,
	                                                       const Points & points2 ,
	                                                       double outlier_threshold ,
	                                                       int num_ransac ) {

		using namespace std;
		using Samples = vector < unsigned long >;

		static mt19937                      mt;
		uniform_int_distribution < size_t > distribution ( 0 , points1.size ( ) - 1 );

		cv::Matx44f matrix;

		int vote_max = 0;

		for ( int i = 0 ; i < num_ransac ; ++i ) {

			Samples samples ( 3 );

			samples[ 0 ] = distribution ( mt );
			samples[ 1 ] = distribution ( mt );
			samples[ 2 ] = distribution ( mt );

			Points _3points1 , _3points2;

//			for_each ( samples.begin ( ) , samples.end ( ) , [ & ] ( const int & sample ) {
//				_3points1.push_back ( points1[ sample ] );
//				_3points2.push_back ( points2[ sample ] );
//			} );

			for ( const auto & sample : samples ) {
				_3points1.push_back ( points1[ sample ] );
				_3points2.push_back ( points2[ sample ] );
			}

			const auto temp_matrix = MapCreator::ComputeTransformationMatrix ( _3points1 , _3points2 );
			const auto errors      = ComputeErrors ( points1 , points2 , temp_matrix );

			int vote = 0;
			for ( const auto & err : errors ) {
				if ( err <= outlier_threshold )
					++vote;
			}

			if ( vote > vote_max ) {
				vote_max = vote;
				matrix   = temp_matrix;
			}
		}

		return std::make_pair ( vote_max , matrix );
	}

	// 初期行列を求める RANSAC FLOW
	std::pair < int , cv::Matx44f > ComputeInitialMatrixWithFlow ( const Points & points1 ,
	                                                               const Points & points2 ,
	                                                               int num_ransac ,
	                                                               double outlier_threshold ) {

		using namespace std;

		using Samples = std::set < unsigned long >;

		//メルセンヌ・ツイスター
		static std::mt19937 mt;

		std::uniform_int_distribution < size_t > distribution ( 0 , points1.size ( ) - 1 );

		cv::Matx44f matrix;
		int         vote_max = 0;

		for ( int i = 0 ; i < num_ransac ; ++i ) {

			// ランダムに3点選ぶ
			Samples samples;

			samples.insert ( distribution ( mt ) ); // sample 1
			samples.insert ( distribution ( mt ) ); // sample 2
			samples.insert ( distribution ( mt ) ); // sample 3

			// ランダムに選んだ3点で変換行列を求める
			Points p1 , p2;

			for ( auto & sample : samples ) {
				p1.push_back ( points1[ sample ] );
				p2.push_back ( points2[ sample ] );
			}

			const auto m = MapCreator::ComputeTransformationMatrix ( p1 , p2 );

			// 対応点ペアのフローを評価する
			const auto flow_errors = ComputeFlowErrors ( points1 , points2 );

			// 変換行列を適用して、エラーを求める
			const auto dist_errors = ComputeErrors ( points1 , points2 , m );

			// エラーが閾値以下だったら投票を行う
			auto vote = 0;

			for ( auto i = 0 ; i < dist_errors.size ( ) ; ++i ) {

				const auto & dist_error = dist_errors[ i ];
				const auto & flow_error = flow_errors[ i ];

				if ( dist_error <= outlier_threshold and flow_errors[ i ] > 0 ) {
					++vote;
				}
			}

			// 投票数が最大のものを仮パラメータとして採用する
			if ( vote > vote_max ) {
				vote_max = vote;
				matrix   = m;
			}
		}

		return std::make_pair ( vote_max , matrix );
	}

}    // namespace

namespace MapCreator {

	// ２つの点群間の変換行列（point1 → points2）を求める
	cv::Matx44f ComputeTransformationMatrix ( const Points & points1 ,
	                                          const Points & points2 ,
	                                          const int num_ransac ,
	                                          const double outlier_threshold ,
	                                          const double inlier_threshold ) {

		using namespace std;

		// 初期行列を求める
		const auto result      = ComputeInitialMatrix ( points1 , points2 , outlier_threshold , num_ransac );
		const auto vote_max    = result.first;
		const auto init_matrix = result.second;
		const auto errors      = ComputeErrors ( points1 , points2 , init_matrix );

		Points inlier1;
		Points inlier2;

		for ( auto i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inlier1.push_back ( points1[ i ] );
				inlier2.push_back ( points2[ i ] );
			}
		}

		// インライアのみで行列を計算する
		return ComputeTransformationMatrix ( inlier1 , inlier2 );
	}

	std::pair < InlierPoints , InlierPoints > ComputeInliers ( const Points & points1 ,
	                                                           const Points & points2 ,
	                                                           const int num_ransac ,
	                                                           const double outlier_threshold ,
	                                                           const double inlier_threshold ) {

		InlierPoints inliers1 , inliers2;

		// 初期行列を求める
		const auto result      = ComputeInitialMatrix ( points1 , points2 , outlier_threshold , num_ransac );
		const auto init_matrix = result.second;

		// 仮パラメータで再計算し、エラーが閾値以下のものを抽出する
		const auto errors = ComputeErrors ( points1 , points2 , init_matrix );

		for ( auto i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inliers1.push_back ( points1[ i ] );
				inliers2.push_back ( points2[ i ] );
			}
		}

		// インライアを返す
		return std::make_pair ( inliers1 , inliers2 );

	};

	std::pair < InlierPoints , InlierPoints > ComputeInliersWithFlow ( const Points & points1 ,
	                                                                   const Points & points2 ,
	                                                                   const int num_ransac ,
	                                                                   const double outlier_threshold ,
	                                                                   const double inlier_threshold ) {

		InlierPoints inliers1 , inliers2;

		// 初期行列を求める
		const auto result      = ComputeInitialMatrixWithFlow ( points1 , points2 , num_ransac , outlier_threshold );
		const auto init_matrix = result.second;

		// 仮パラメータで再計算し、エラーが閾値以下のものを抽出する
		const Errors errors = ComputeErrors ( points1 , points2 , init_matrix );

		for ( size_t i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inliers1.push_back ( points1[ i ] );
				inliers2.push_back ( points2[ i ] );
			}
		}

		cv::Point3f _mean1;
		cv::Point3f _mean2;

		std::for_each ( inliers1.begin ( ) , inliers1.end ( ) , [ & ] ( const cv::Point3f & point ) -> void { _mean1 += point; } );
		std::for_each ( inliers2.begin ( ) , inliers2.end ( ) , [ & ] ( const cv::Point3f & point ) -> void { _mean2 += point; } );

		_mean1.x /= inliers1.size ( );
		_mean1.y /= inliers1.size ( );
		_mean1.z /= inliers1.size ( );

		_mean2.x /= inliers2.size ( );
		_mean2.y /= inliers2.size ( );
		_mean2.z /= inliers2.size ( );

		auto centroid_diff = _mean1 - _mean2;

		std::cout << _mean1 << ", " << _mean2 << std::endl;

		Errors flow_errors;
		flow_errors.resize ( inliers1.size ( ) );

		for ( auto i = 0 ; i < flow_errors.size ( ) ; ++i ) {

			const auto & p1 = inliers1[ i ];
			const auto & p2 = inliers2[ i ] + centroid_diff;

			glm::vec3 vec1 ( p1.x , p1.y , p1.z );
			glm::vec3 vec2 ( p2.x , p2.y , p2.z );

			float _val1 = glm::dot ( vec2 , vec1 );
			float _val2 = glm::length ( vec1 ) * glm::length ( vec2 );

			auto _angle = acosf ( _val1 / _val2 ) / glm::pi < float > ( ) * 180.0f;

			auto _cross_product = glm::cross ( vec2 , vec1 );
			auto _val3          = glm::dot ( _cross_product , glm::vec3 ( 0.0f , 1.0f , 0.0f ) );

			int sgn = ( _val3 > 0.0f ) ? ( 1 ) : ( -1 );

			// const auto angle = glm::angle ( vec1 , vec2 ); // ERROR???
			// const auto __angle = glm::orientedAngle ( vec1 , vec2 , glm::vec3 ( 0.0f , 1.0f , 0.0f ) ); // ERROR??
			// std::cout << __angle << std::endl;

			flow_errors[ i ] = _angle;
//			flow_errors[ i ] = sgn * _angle;

		}


		// インライアを返す
		return std::make_pair ( inliers1 , inliers2 );

	};

	cv::Matx44f ComputeTransformationMatrix ( const Points & points1 , const Points & points2 ) {

		using namespace std;

		TransformationFromCorrespondences < double > tfc;    // float では問題が発生した。本当だ！（リンクンより）回転行列が NaN になっちゃう

		for ( auto i = 0 ; i < points1.size ( ) ; ++i ) {
			const auto & from = points1[ i ];
			const auto & to   = points2[ i ];

			tfc.Add ( from , to );
		}

		return tfc.GetTransformation ( );
	}
}
