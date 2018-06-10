//
// Created by LinKun on 11/6/15.
//

#include <QApplication>
#include <QSurfaceFormat>

#include <MapCreator/MainWindow.h>

int main ( int argc , char * argv[] ) {

	QApplication app ( argc , argv );

	QSurfaceFormat f;
	f.setProfile ( QSurfaceFormat::CoreProfile );
	f.setVersion ( 4 , 1 );
	f.setSwapBehavior ( QSurfaceFormat::DoubleBuffer );
	f.setRenderableType ( QSurfaceFormat::OpenGL );
	// f.setSamples (4);
	QSurfaceFormat::setDefaultFormat ( f );


	NiS::MainWindow w;
	w.show ( );

	return app.exec ( );
}