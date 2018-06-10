////
//// Created by LinKun on 10/25/15.
////
//
//#ifndef NIS_GRABBER_H
//#define NIS_GRABBER_H
//
//#include <QWidget>
//#include <QDir>
//
//#include <Serialize.h>
//
//class Grabber : public QObject
//{
//Q_OBJECT
//
//public:
//
//	struct Property
//	{
//		std::string vendor;     ///< メーカー名
//		std::string name;       ///< デバイス名
//
//		float horizontal_fov;   ///< 横方向の視野角（ラジアン）
//		float vertical_fov;     ///< 縦方向の視野角（ラジアン）
//		int   width;            ///< 画面の横サイズ
//		int   height;           ///< 画面の縦サイズ
//
//		Property ( ) :
//				horizontal_fov ( 1.0225999f ) ,
//				vertical_fov ( 0.79661566f ) ,
//				width ( 640 ) ,
//				height ( 480 ) { }
//	};
//
//	Grabber ( QWidget * parent = 0 ) { };
//	Grabber ( const QDir & dir , QWidget * parent = 0 ) { };
//
//private:
//
//	friend class boost::serialization::access;
//
//	void serialize ( Archive & ar , unsigned int version ) {
//
//		ar & vender;
//		ar & name;
//
//		uint64 focal_length;
//		double pixel_size;
//
//		if ( version == 0 ) {
//			ar & focal_length;
//			ar & pixel_size;
//		}
//		else {
//			ar & horizontal_fov;
//			ar & vertical_fov;
//		}
//
//		ar & width;
//		ar & height;
//	}
//
//	QDir data_dir_;
//};
//
//
//#endif //NIS_GRABBER_H
