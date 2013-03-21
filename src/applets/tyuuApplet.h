#ifndef TYUUAPPLET_H
#define TYUUAPPLET_H

#include "applet.h"

#define TYUU_NAME "tyuu"
#define TYUU_TITLE "Tyuu"

class tyuuApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    tyuuApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame ) {}
};

#endif // TYUUAPPLET_H
