#include "pluginApplet.h"


/**
 *
 */
void PluginApplet::setFrame( cv::Mat& frame )
{

    // received a frame
    //qDebug() << "PluginApplet" << "Received a frame";
}

Q_EXPORT_PLUGIN2(pluginapplet, PluginApplet);
