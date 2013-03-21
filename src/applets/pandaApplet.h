#ifndef PANDAAPPLET_H
#define PANDAAPPLET_H

#include "applet.h"

#define PANDA_NAME "panda"
#define PANDA_TITLE "Panda"

class pandaApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    pandaApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame ) {}
};

#endif // PANDAAPPLET_H
