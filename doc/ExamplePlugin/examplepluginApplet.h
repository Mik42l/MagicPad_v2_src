
#ifndef EXAMPLEPLUGINAPPLET_H
#define EXAMPLEPLUGINAPPLET_H

#include <QLabel>
#include <QWidget>

#include "applets/appletInterface.h"

class ExamplePluginApplet : public QWidget, AppletInterface
{
    Q_OBJECT
    Q_INTERFACES(AppletInterface)

public:

    static const QString TAG;

    ExamplePluginApplet();

    virtual QString name() { return "pluginexample"; }
    virtual QString title() { return "Plugin Example"; }
    virtual QPixmap icon() { return QPixmap(":/icon_question.png"); }

    virtual QString descriptionText() { return "This plugins example shows how to add an external plugin to the MagicPad software."; }
    virtual QString marketingText() { return "Plugins are a very convenient way to add fonctionnalities over the mainstream MagicPad software."; }
    virtual QString technicalText() { return "Please refer to the documentation for more information about how to create your how plugins."; }

    virtual void hide() { QWidget::hide(); }
    virtual void resize( const QSize& size ) { QWidget::resize( size ); }
    virtual void show() { QWidget::show(); }
    virtual void start() {}
    virtual void stop() {}
    virtual void setParent( QWidget* parent ) { QWidget::setParent( parent ); }
    virtual void setFrame( cv::Mat& frame );
		
signals:

    void meanChanged( double mean );

private:
	QLabel *mMeanValue;
};

#endif // EXAMPLEPLUGINAPPLET_H
