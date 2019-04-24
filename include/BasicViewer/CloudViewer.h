//
// Created by LinKun on 10/28/15.
//

#ifndef MAPCREATOR_CLOUDVIEWER_H
#define MAPCREATOR_CLOUDVIEWER_H

#include "BaseViewer.h"

namespace MapCreator {
	class CloudViewer : public BaseViewer
	{
	Q_OBJECT

	public:

		void initializeGL ( ) override;
		void paintGL ( ) override;
		void resizeGL ( int w , int h ) override;

	};
}


#endif //MAPCREATOR_CLOUDVIEWER_H
