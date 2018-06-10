//
// Created by LinKun on 10/28/15.
//

#ifndef NIS_EVALUTATIONVIEWER_H
#define NIS_EVALUTATIONVIEWER_H


#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLWidget>

#include "../../../bin/lib/BasicViewer/ui_EvaluationViewer.h"

namespace Ui {

    class EvaluationViewer;
}

namespace NiS {

    class EvaluationViewer : public QOpenGLWidget {

        Q_OBJECT

    public:

        EvaluationViewer(QWidget *parent = 0);

        ~EvaluationViewer() { }

    private:

        Ui::EvaluationViewer ui_;
    };
}


#endif //NIS_EVALUTATIONVIEWER_H
