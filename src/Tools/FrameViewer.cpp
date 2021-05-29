//
// Created by LinKun on 11/15/15.
//

#include <QApplication>

#include "BasicViewer/FrameViewer.h"

int main ( int argc , char ** argv ) {

	QApplication app ( argc , argv );

	MapCreator::FrameViewer viewer;

	viewer.show ( );

	return app.exec ( );

}
