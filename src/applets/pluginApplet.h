
#ifndef PLUGINAPPLET_H
#define PLUGINAPPLET_H

#include "appletInterface.h"

class PluginApplet : public QWidget, AppletInterface
{
    Q_OBJECT
    Q_INTERFACES(AppletInterface)

public:

    virtual QString name() { return "pluginApplet"; }
    virtual QString title() { return "Plugin Applet"; }
    virtual QPixmap icon() { return QPixmap(); }

    virtual QString descriptionText() { return "...description text..."; }
    virtual QString marketingText() { return "...marketing text..."; }
    virtual QString technicalText() { return "...technical text..."; }

    virtual void setParent(QWidget *parent) {
        QWidget::setParent( parent );
    }

    virtual void hide() { QWidget::hide(); }

    virtual void resize( const QSize& size ) { QWidget::resize( size ); }

    virtual void show() { QWidget::show(); }

    virtual void start() {}

    virtual void stop() {}

    virtual void setFrame( cv::Mat& frame );

 };

#endif // PLUGINAPPLET_H
