//
// Created by LinKun on 9/13/15.
//

#include "SLAM/Transformation.h"

#include <random>
#include <fstream>

#include <opencv2/opencv.hpp>

namespace {

	using Points = std::vector < cv::Point3f >;
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

		void Add ( const cv::Point3f & point1 , const cv::Point3f & point2 , T weight = 1.0 ) {

			if ( weight == 0 ) {
				return;
			}

			++samples_;
			accumulated_weight_ += weight;
			const T alpha = weight / accumulated_weight_;

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


	bool EvaluateTriangle ( const Points & triangle , float th_1 , float th_2 , float th_3 ) {

		cv::Mat triangle_mat ( ( int ) triangle.size ( ) , 3 , CV_32FC1 , ( void * ) triangle.data ( ) );
		cv::PCA pca ( triangle_mat , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

		float _1st_pc = pca.eigenvalues.at < float > ( 0 , 0 );
		float _2nd_pc = pca.eigenvalues.at < float > ( 1 , 0 );
		float _3rd_pc = pca.eigenvalues.at < float > ( 2 , 0 );


		return _2nd_pc > _1st_pc / 1.5;

//		return _2nd_pc > th_3;

	}

	// Evaluate the shape of 2 triangles
	bool EvaluateTriangle ( const Points & triangle1 , const Points & triangle2 ) {

		cv::Mat triangle_mat1 ( ( int ) triangle1.size ( ) , 3 , CV_32FC1 , ( void * ) triangle1.data ( ) );
		cv::Mat triangle_mat2 ( ( int ) triangle2.size ( ) , 3 , CV_32FC1 , ( void * ) triangle2.data ( ) );

		cv::PCA pca1 ( triangle_mat1 , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );
		cv::PCA pca2 ( triangle_mat2 , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

		float tri1_1st_pc = pca1.eigenvalues.at < float > ( 0 , 0 );
		float tri1_2nd_pc = pca1.eigenvalues.at < float > ( 1 , 0 );

		float tri2_1st_pc = pca2.eigenvalues.at < float > ( 0 , 0 );
		float tri2_2nd_pc = pca2.eigenvalues.at < float > ( 1 , 0 );

		float ratio1 = ( tri1_1st_pc / tri2_1st_pc );
		float ratio2 = ( tri1_2nd_pc / tri2_2nd_pc );

		return 0.9f < ratio1 and ratio1 < 1.0f and 0.9f < ratio2 and ratio2 < 1.0f;
	}


	// 初期行列を求める
	std::pair < int , cv::Matx44f > ComputeInitialMatrix ( const Points & points1 ,
	                                                       const Points & points2 ,
	                                                       double outlier_threshold ,
	                                                       int count ) {

		using namespace std;

		typedef std::set < int > Samples;

		//メルセンヌ・ツイスター
		static std::mt19937                      mt;
		std::uniform_int_distribution < size_t > distribution ( 0 , points1.size ( ) - 1 );

		cv::Matx44f matrix;
		int         vote_max = 0;

		for ( int i = 0 ; i < count ; ++i ) {

			// ランダムに3点選ぶ
			Samples samples;

			while ( samples.size ( ) < 3 ) {
				size_t index = distribution ( mt );
				samples.insert ( index );
			}

			// ランダムに選んだ3点で変換行列を求める
			Points p1 , p2;

			for ( auto sample = samples.begin ( ) ; sample != samples.end ( ) ; ++sample ) {
				p1.push_back ( points1[ * sample ] );
				p2.push_back ( points2[ * sample ] );
			}

			// Triangle evalutation

			const cv::Matx44f m = NiS::ComputeTransformationMatrix ( p1 , p2 );

			// 変換行列を適用して、エラーを求める
			const Errors errors = ComputeErrors ( points1 , points2 , m );

			// エラーが閾値以下だったら投票を行う
			int vote = 0;
			for ( const auto & err : errors ) {
				if ( err <= outlier_threshold ) {
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

	// With evalutation of triangle
	std::pair < int , cv::Matx44f > ComputeInitialMatrix ( const Points & points1 ,
	                                                       const Points & points2 ,
	                                                       double outlier_threshold ,
	                                                       int count ,
	                                                       float th_1 ,
	                                                       float th_2 ,
	                                                       float th_3 ) {

		using namespace std;

		typedef std::set < int > Samples;

		//メルセンヌ・ツイスター
		static std::mt19937                      mt;
		std::uniform_int_distribution < size_t > distribution ( 0 , points1.size ( ) - 1 );

		cv::Matx44f matrix;
		int         bin_max = 0;

		for ( int i = 0 ; i < count ; ++i ) {

			// ランダムに3点選ぶ
			Samples samples;

			while ( samples.size ( ) < 3 ) {
				size_t index = distribution ( mt );
				samples.insert ( index );
			}

			// ランダムに選んだ3点で変換行列を求める
			Points p1 , p2;

			for ( auto sample = samples.begin ( ) ; sample != samples.end ( ) ; ++sample ) {
				p1.push_back ( points1[ * sample ] );
				p2.push_back ( points2[ * sample ] );
			}

			// Triangle evalutation
			if ( !EvaluateTriangle ( p1 , p2 ) ) {
				continue;
			}

			const cv::Matx44f m = NiS::ComputeTransformationMatrix ( p1 , p2 );

			// 変換行列を適用して、エラーを求める
			const Errors errors = ComputeErrors ( points1 , points2 , m );

			// エラーが閾値以下だったら投票を行う
			int bin = 0;
			for ( const auto & err : errors ) {
				if ( err <= outlier_threshold ) {
					++bin;
				}
			}

			// 投票数が最大のものを仮パラメータとして採用する
			if ( bin > bin_max ) {
				bin_max = bin;
				matrix  = m;
			}
		}

		std::cout << "RANSAC - #max_vote / #matches  : " << bin_max << " / " << points1.size ( ) << std::endl;

		return std::make_pair ( bin_max , matrix );
	}

	cv::Matx44f ComputeInitialMatrix ( const Points & points1 , const Points & points2 , int num_ransac ,
	                                   double outlier_threshold ) {

		using namespace std;

		using Result = std::pair < int , cv::Matx44f >;

		const int count       = 1;
		const int trial_count = num_ransac / count;

		std::vector < Result > results ( count );

//		cv::Mat_ < float > points1_mat ( points1.size ( ) , 3 );
		cv::Mat points1_mat ( points1.size ( ) , 3 , CV_32FC1 , ( void * ) ( points1.data ( ) ) );

//
//		for ( auto i = 0 ; i < points1.size ( ) ; ++i ) {
//			points1_mat ( i , 0 ) = points1[ i ].x;
//			points1_mat ( i , 1 ) = points1[ i ].y;
//			points1_mat ( i , 2 ) = points1[ i ].z;
//		}

		cv::PCA pca ( points1_mat , cv::Mat ( ) , CV_PCA_DATA_AS_ROW );

//		for ( auto i = 0 ; i < points1_mat.rows ; ++i ) {
//			cout << points1_mat ( i , 0 ) << ", " << points1_mat ( i , 1 ) << ", " << points1_mat ( i , 2 ) << std::endl;
//		}

		////////////////
//		float th_1 = pca.eigenvalues.at < float > ( 0 , 0 );
//		float th_2 = pca.eigenvalues.at < float > ( 1 , 0 );
//		float th_3 = pca.eigenvalues.at < float > ( 2 , 0 );
//
//		std::cout << "eigenvalues of original points : " << th_1 << ", " << th_2 << ", " << th_3 << std::endl;
//		std::cout << "eigenmatrix of original points : " << std::endl;
//		cout << pca.eigenvectors << std::endl;

		////////////////

		for ( int i = 0 ; i < count ; ++i ) {
//			results[ i ] = ComputeInitialMatrix ( points1 , points2 , outlier_threshold , trial_count , th_1 , th_2 , th_3 );
			results[ i ] = ComputeInitialMatrix ( points1 , points2 , outlier_threshold , trial_count );
		}

		int vote_max = 0;

		cv::Matx44f m = cv::Matx44f::eye ( );

		for ( Result & result : results ) {

			if ( result.first > vote_max ) {

				vote_max = result.first;
				m        = result.second;
			}
		}

		return m;
	}


}    // namespace

namespace NiS {

	// ２つの点群間の変換行列（point1 → points2）を求める
	cv::Matx44f ComputeTransformationMatrix ( const Points & points1 ,
	                                          const Points & points2 ,
	                                          int num_ransac ,
	                                          double outlier_threshold ,
	                                          double inlier_threshold ) {

		using namespace std;

		// 初期行列を求める
		const cv::Matx44f init_matrix = ComputeInitialMatrix ( points1 , points2 , num_ransac , outlier_threshold );


		// 仮パラメータで再計算し、エラーが閾値以下のものを抽出する
		const Errors errors = ComputeErrors ( points1 , points2 , init_matrix );

		Points inlier1 , inlier2;

		for ( size_t i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inlier1.push_back ( points1[ i ] );
				inlier2.push_back ( points2[ i ] );
			}
		}

		// インライアのみで行列を計算する
		return ComputeTransformationMatrix ( inlier1 , inlier2 );
	}

	cv::Matx44f ComputeTransformationMatrix ( const Points & points1 ,
	                                          const Points & points2 ,
	                                          int num_ransac ,
	                                          double outlier_threshold ,
	                                          double inlier_threshold ,
	                                          std::vector < cv::Point3f > & inliers1 ,
	                                          std::vector < cv::Point3f > & inliers2 ) {

		std::ofstream out ( "C:/Dev/Data/points.txt" );

		for ( int i = 0 ; i < points1.size ( ) ; ++i ) {
			out << points1[ i ].x << " " << points1[ i ].y << " " << points1[ i ].z << " " << points2[ i ].x << " " <<
			points2[ i ].y << " " << points2[ i ].z << std::endl;
		}

		out.close ( );

		// 初期行列を求める
		const cv::Matx44f init_matrix = ComputeInitialMatrix ( points1 , points2 , num_ransac , outlier_threshold );

		// 仮パラメータで再計算し、エラーが閾値以下のものを抽出する
		const Errors errors = ComputeErrors ( points1 , points2 , init_matrix );

		for ( size_t i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inliers1.push_back ( points1[ i ] );
				inliers2.push_back ( points2[ i ] );
			}
		}

		// インライアのみで行列を計算する
		return ComputeTransformationMatrix ( inliers1 , inliers2 );
	}

	std::pair < InlierPoints , InlierPoints > ComputeInliers ( const Points & points1 ,
	                                                           const Points & points2 ,
	                                                           const int num_ransac ,
	                                                           const double outlier_threshold ,
	                                                           const double inlier_threshold ) {

		InlierPoints inliers1 , inliers2;

		// 初期行列を求める
		const cv::Matx44f init_matrix = ComputeInitialMatrix ( points1 , points2 , num_ransac , outlier_threshold );

		// 仮パラメータで再計算し、エラーが閾値以下のものを抽出する
		const Errors errors = ComputeErrors ( points1 , points2 , init_matrix );

		for ( size_t i = 0 ; i < errors.size ( ) ; ++i ) {
			if ( errors[ i ] <= inlier_threshold ) {
				inliers1.push_back ( points1[ i ] );
				inliers2.push_back ( points2[ i ] );
			}
		}

		// インライアを返す
		return std::make_pair ( inliers1 , inliers2 );

	};

	cv::Matx44f ComputeTransformationMatrix ( const Points & points1 , const Points & points2 ) {

		using namespace std;

		TransformationFromCorrespondences < float > tfc;    // float では問題が発生した

		for ( size_t i = 0 ; i < points1.size ( ) ; ++i ) {
			const auto & from = points1[ i ];
			const auto & to   = points2[ i ];

			tfc.Add ( from , to , 1.0f );
		}

		return tfc.GetTransformation ( );
	}
}
