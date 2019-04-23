//
// Created by LinKun on 10/28/15.
//

#ifndef NIS_CLOUDVIEWER_H
#define NIS_CLOUDVIEWER_H

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


#endif //NIS_CLOUDVIEWER_H
