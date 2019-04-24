//
// Created by LinKun on 10/28/15.
//

#ifndef MAPCREATOR_EVALUTATIONVIEWER_H
#define MAPCREATOR_EVALUTATIONVIEWER_H


#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLWidget>

#include "../../../bin/lib/BasicViewer/ui_EvaluationViewer.h"

namespace Ui {

    class EvaluationViewer;
}

namespace MapCreator {

    class EvaluationViewer : public QOpenGLWidget {

        Q_OBJECT

    public:

        EvaluationViewer(QWidget *parent = 0);

        ~EvaluationViewer() { }

    private:

        Ui::EvaluationViewer ui_;
    };
}


#endif //MAPCREATOR_EVALUTATIONVIEWER_H
