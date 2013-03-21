#include "examplepluginApplet.h"

const QString ExamplePluginApplet::TAG = QString("ExamplePluginApplet");

/**
 *
 */
ExamplePluginApplet::ExamplePluginApplet()
{
    qDebug() << TAG << "constructor";
}

/**
 *
 */
void ExamplePluginApplet::setFrame(cv::Mat &frame)
{
    qDebug() << TAG << "in set frame";
}

Q_EXPORT_PLUGIN2(examplePluginApplet, ExamplePluginApplet);
