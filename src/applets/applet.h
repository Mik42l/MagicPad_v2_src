#ifndef APPLET_H
#define APPLET_H

#include <QWidget>
#include "QsLog.h"

#include <opencv2/opencv.hpp>

#include "appletInterface.h"


/**
 *
 */
class Applet : public QWidget, public AppletInterface
{
    Q_OBJECT
    Q_INTERFACES( AppletInterface )

public:

    //const static char* gestureName[] = {"TWIST"};

    static const QString TAG;

    Applet(QWidget *parent = 0);

    // Applet identity
    virtual QString name() { return mName; }

    virtual QString title() { return mTitle; }

    virtual QPixmap icon();

    // Descriptive text
    virtual QString descriptionText() { return mDescription; }

    virtual QString marketingText() { return mMarketing; }

    virtual QString technicalText() { return mTechnical; }

    // Accepted gestures
    virtual void acceptedGestures(QPixmap * ges);

    virtual void setGestures(GestureType ges0, GestureType ges1 = NONE, GestureType ges2 = NONE);

    void setFrame( cv::Mat& frame ) = 0;

    virtual void hide() {
        QWidget::hide();
    }

    virtual void resize( const QSize& size ) {
        QWidget::resize( size );
    }

    virtual void show() {
        QWidget::show();
    }

    virtual void start() {}
    virtual void stop() {}

    virtual void setParent(QWidget *parent) {
        QWidget::setParent( parent );
    }

protected:

    void loadTextFromXml();

protected:
    QString mName;      // short name: no spaces
    QString mTitle;     // title displayed on home screen
    GestureType mGestures[ APPLET_INTERFACE_GESTURE_NUMBER];  // image showing accepted gestures

    // Descriptive text
    QString mDescription;   // Complete description
    QString mMarketing;     // Advantages over competing products
    QString mTechnical;     // Technical details
};

#endif // APPLET_H
