#ifndef TEMPLATEAPPLET_H
#define TEMPLATEAPPLET_H

#include "applet.h"

#define TEMPLATEAPPLET_NAME "templateapplet"
#define TEMPLATEAPPLET_TITLE "Template Applet"

class TemplateApplet : public Applet
{
    Q_OBJECT

public:

    TemplateApplet(QWidget *parent = 0);

public slots:

    void setFrame(cv::Mat& frame) {}
};

#endif // TEMPLATEAPPLET_H
