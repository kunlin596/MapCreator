//
// Created by LinKun on 9/12/15.
//

#ifndef LK_SLAM_SERIALIZE_H
#define LK_SLAM_SERIALIZE_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <glm/glm.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/split_member.hpp>

#include <glm/gtc/type_ptr.hpp>

BOOST_SERIALIZATION_SPLIT_FREE( cv::Mat )

namespace boost {

	namespace serialization {

		// cv::Mat
		template < class Archive >
		void save ( Archive & ar , const cv::Mat & m , const unsigned int version ) {

			size_t elem_size = m.elemSize ( );
			size_t elem_type = m.type ( );
			ar & m.cols;
			ar & m.rows;
			ar & elem_size;
			ar & elem_type;
			const size_t data_size = m.cols * m.rows * elem_size;
			ar & boost::serialization::make_array ( m.ptr ( ) , data_size );
		}


		template < class Archive >
		void load ( Archive & ar , cv::Mat & m , const unsigned int version ) {

			int    cols , rows;
			size_t elem_size , elem_type;
			ar & cols;
			ar & rows;
			ar & elem_size;
			ar & elem_type;

			m.create ( rows , cols , static_cast<int>(elem_type) );
			const size_t data_size = m.cols * m.rows * elem_size;
			ar & boost::serialization::make_array ( m.ptr ( ) , data_size );
		}


		// cv::Point_<T>
		template < class Archive , typename T >
		void serialize ( Archive & ar , cv::Point_ < T > & pt , unsigned int version ) {

			ar & pt.x;
			ar & pt.y;
		}


		// cv::Point3_<T>
		template < class Archive , typename T >
		void serialize ( Archive & ar , cv::Point3_ < T > & pt , unsigned int version ) {

			ar & pt.x;
			ar & pt.y;
			ar & pt.z;
		}


		// cv::KeyPoint
		template < class Archive >
		void serialize ( Archive & ar , cv::KeyPoint & keypoint , unsigned int version ) {

			ar & keypoint.angle;
			ar & keypoint.class_id;
			ar & keypoint.octave;
			ar & keypoint.pt;
			ar & keypoint.response;
			ar & keypoint.size;
		}


		// cv::Vec<T, N>
		template < class Archive , typename T , int N >
		void serialize ( Archive & ar , cv::Vec < T , N > & vec , unsigned int version ) {

			for ( int i = 0 ; i < N ; ++i ) {
				ar & vec ( i );
			}
		}


		// cv::Matx<T, M, N>
		template < class Archive , typename T , int M , int N >
		void serialize ( Archive & ar , cv::Matx < T , M , N > & m , unsigned int version ) {

			ar & boost::serialization::make_array ( m.val , M * N );
		}


		template < class Archive >
		void serialize ( Archive & ar , glm::tvec3 < float , glm::precision::highp > & vec , unsigned int version ) {

			ar & vec.x;
			ar & vec.y;
			ar & vec.z;
		}

		template < class Archive >
		void serialize ( Archive & ar , glm::tvec4 < float , glm::precision::highp > & vec , unsigned int version ) {

			ar & vec.x;
			ar & vec.y;
			ar & vec.z;
			ar & vec.w;
		}

		template < class Archive >
		void serialize ( Archive & ar , glm::tmat4x4 < float , glm::precision::highp > & m , unsigned int version ) {

			ar & m[ 0 ];
			ar & m[ 1 ];
			ar & m[ 2 ];
			ar & m[ 3 ];
		}

		template < class Archive >
		void serialize ( Archive & ar ,
		                 std::pair < std::vector < size_t > , std::vector < glm::tmat4x4 < float , glm::precision::highp > > > & matrices_info ,
		                 unsigned int version ) {

			ar & matrices_info.first;
			ar & matrices_info.second;

		};

	}

}

namespace NiS {
	// Xtion data frame definitions

	struct RawDataFrame
	{
		using ColorImage = cv::Mat_ < cv::Vec3b >;
		using DepthImage = cv::Mat_ < ushort >;

		ColorImage  color_image;
		DepthImage  depth_image;
		std::string name;
		int         id;
	};


	using RawDataFrames = std::vector < RawDataFrame >;


// Parameters for reading AIST image data
	const std::string kRawDataFrameHeader = "XTION DATA";

	const int kImageRows          = 480;
	const int kImageCols          = 640;
	const int kColorImageChannels = 3;
	const int kDepthImageChannels = 1;

	const int kColorImageByteSize = kImageCols * kImageRows * kColorImageChannels * sizeof ( char );
	const int kDepthImageByteSize = kImageCols * kImageRows * kDepthImageChannels * sizeof ( ushort );

	const int kSkipLengthOfColorImage =
			          sizeof ( char[10] ) +                    // sensor type
			          sizeof ( int ) +                            // version
			          sizeof ( int ) +                            // device count
			          sizeof ( kImageRows ) +                    // color image rows
			          sizeof ( kImageCols ) +                    // color image cols
			          sizeof ( int );                            // color image type (OpenCV)


	const int kSkipLengthOfDepthImage =
			          sizeof ( kImageRows ) +                    // depth image rows
			          sizeof ( kImageCols ) +                    // depth image cols
			          sizeof ( int );                            // depth image type (OpenCV)




	//----------------------------------------------
	// Read Image Data

	// Read primitive data type from stream
	// Example : int, float, double, etc.
	template < class T >
	inline T Read ( std::istream & in ) {

		T value;
		in.read ( reinterpret_cast< char * >( & value ) , sizeof ( value ) );
		return value;
	}

	// Read std::string from stream
	inline std::string ReadString ( std::istream & in , unsigned int size ) {

		std::string str;
		str.resize ( size );
		in.read ( const_cast< char * >( str.data ( )) , size );
		return str;
	}

	// Read std::vector from stream
	// T is for the data type of the contents of vector
	template < class T >
	inline std::vector < T > ReadVector ( std::istream & in ) {

		std::vector < T > vec;
		const int         size = Read < int > ( in );
		if ( size > 0 ) {
			vec.resize ( size );
			in.read ( reinterpret_cast< char * >( vec.data ( )) , sizeof ( T ) * size );
		}
		return vec;
	}

//    template<>
//    inline std::string Read(std::istream &in) {
//        const unsigned int size = Read<unsigned int>(in);
//        return ReadString(in, size);
//    }


	// Read cv::Mat from stream
	template < >
	inline cv::Mat Read ( std::istream & in ) {

		const int rows = Read < int > ( in );
		const int cols = Read < int > ( in );
		const int type = Read < int > ( in );
		cv::Mat   m;
		if ( rows * cols > 0 ) {
			m.create ( rows , cols , type );
			in.read ( reinterpret_cast< char * >( m.data ) , m.elemSize ( ) * rows * cols );
		}
		return m;
	}

	template < class T >
	inline T Read ( std::istream & in , int version ) {

		return T ( );
	}

	template < >
	inline RawDataFrame Read ( std::istream & in , int version ) {

		RawDataFrame frame;
		frame.color_image = Read < cv::Mat > ( in );
		frame.depth_image = Read < cv::Mat > ( in );
		return frame;
	}


	// Read multiple frames from stream
	template < >
	inline RawDataFrames Read ( std::istream & in ) {

		RawDataFrames frames;

		const std::string head = ReadString ( in , static_cast<unsigned int>( kRawDataFrameHeader.size ( )) );

		if ( head == kRawDataFrameHeader ) {

			const int version = Read < int > ( in );
			const int count   = Read < int > ( in );

			frames.resize ( count );

			for ( RawDataFrame & frame : frames ) {
				frame = Read < RawDataFrame > ( in , version );
			}
		}

		return frames;
	}

	//----------------------------------------------
	// Write Image Data

	template < class T >
	inline void Write ( std::ostream & out , T value ) {

		out.write ( reinterpret_cast< const char * >( & value ) , sizeof ( value ) );
	}

	template < >
	inline void Write ( std::ostream & out , const std::string & str ) {

		const unsigned int size = static_cast< unsigned int >( str.size ( ));
		Write ( out , size );
		out.write ( str.data ( ) , size );
	}

	template < >
	inline void Write ( std::ostream & out , const cv::Mat & m ) {

		const int rows = m.rows;
		const int cols = m.cols;
		const int type = m.type ( );
		Write ( out , rows );
		Write ( out , cols );
		Write ( out , type );
		if ( rows * cols > 0 ) {
			out.write ( reinterpret_cast< const char * >( m.data ) , m.elemSize ( ) * rows * cols );
		}
	}

	template < class T >
	inline void WriteVector ( std::ostream & out , const std::vector < T > & vec ) {

		Write < int > ( out , static_cast< int >( vec.size ( )) );
		if ( !vec.empty ( ) ) {
			out.write ( reinterpret_cast< const char * >( vec.data ( )) , sizeof ( T ) * vec.size ( ) );
		}
	}
}

#endif //LK_SLAM_SERIALIZE_H
