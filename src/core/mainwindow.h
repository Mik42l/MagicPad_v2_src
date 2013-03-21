#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QSettings>
#include <QWidget>

#include "version.h"

// Debug
#include "QsLog.h"

// Core
#include "actionButton.h"
#include "appletButton.h"
#include "descriptionText.h"
#include "loggerDialog.h"

// Com
#include "com/magicPad.h"
#include "com/frameProducer.h"

// Applets
#include "applets/alphabetApplet.h"
#include "applets/backlightApplet.h"
#include "applets/bargraphApplet.h"
#include "applets/bulbApplet.h"
#include "applets/DjApplet.h"
#include "applets/gestureApplet.h"
#include "applets/imagedisplayApplet.h"
#include "applets/mapsApplet.h"
#include "applets/musicApplet.h"
#include "applets/opdApplet.h"
#include "applets/pongApplet.h"
#include "applets/pictureflowApplet.h"
#include "applets/purpleApplet.h"
#include "applets/rollingballApplet.h"
#include "applets/slideshowApplet.h"
#include "applets/surfaceApplet.h"
#include "applets/switchApplet.h"
#include "applets/twistApplet.h"
#include "applets/vumeterApplet.h"
#include "applets/newprojectApplet.h"

#define MAINWINDOW_APPLETGRID_NCOL  5

#define MAINWINDOW_TEXT_WIDTH 400

#define APPLICATION_SETTINGS_FILE "config.ini"

/**
 *
 */
class View : public QGraphicsView
{

     Q_OBJECT

public:

     View(QGraphicsScene *scene) : QGraphicsView(scene)
     {
         //setViewport( new QGLWidget() );

         setViewportUpdateMode( QGraphicsView::FullViewportUpdate );
         //setViewportUpdateMode( QGraphicsView::BoundingRectViewportUpdate );

         // Anti-aliasing
         setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

         // (0,0) in the top left corner
         setAlignment(Qt::AlignLeft | Qt::AlignTop);

         // background image
         _bg.load(":image/metal.jpg");
         setBackgroundBrush( _bg );
         setCacheMode(QGraphicsView::CacheBackground);

         // force the background to redraw
         QTimer *timer = new QTimer( this );
         timer->setInterval( 200 );
         connect( timer, SIGNAL( timeout() ), this, SLOT( repaint()) );
         timer->start();
     }

 protected:

     void resizeEvent(QResizeEvent *event)
     {
        QGraphicsView::resizeEvent(event);
        fitInView(sceneRect(), Qt::KeepAspectRatio);
     }

     QPixmap _bg;
 };

class MainWindow : public QWidget
{

    Q_OBJECT

public:

    static const QString TAG;

    MainWindow(QWidget *parent = 0);

protected:

    void keyPressEvent(QKeyEvent *e);

    void resizeEvent(QResizeEvent *event)
    {
        int sz;
        if( mShowText ) {
            sz = width() - 510;
        } else {
            sz = width() - 200;
            sz = qMin( 1.3*(height()-50), (double)sz );
        }
        mAppletRect->resize( sz, height()-50 );

        if(mCurrentApplet)
            mCurrentApplet->resize( mAppletRect->size() );
    }


public slots:

    void closeCurrentApplet();

    void closeCurrentAppletLater();

    void launchApplet(AppletInterface *applet);

    void resizeApplets();

signals:

    void goApplet();
    void goAppletWithText();
    void goAppletWithoutText();

    void emulateBackButton();

private:
    void loadApplets(QGraphicsScene *scene);

    void loadSettings();

    void registerApplet(AppletInterface *applet);

    void setupUI();

private slots:

    void dispatchFrame( cv::Mat& frame ) {
        if( mCurrentApplet ) mCurrentApplet->setFrame( frame );
    }

    void changeText();

private:
    // User interface:
    QGraphicsWidget *mAppletButtonGrid;
    QWidget *mAppletRect;
    QGraphicsTextItem *mText;
    QVector<QGraphicsPixmapItem*> mAcceptedGestures;
    QGraphicsWidget *mTextAnsGesturesGroup;
    QRectF mScreen;
    LoggerDialog *mLogger;
    int mDescriptionTextWidth;

    bool mReadyToLaunchApplet;
    AppletInterface *mCurrentApplet;
    FrameProducer *mProducer;
    QList< AppletInterface* > mApplets;
    QState *rootState;

    // Settings
    QSettings *mSettings;
    bool mShowLogger;
    bool mShowBack;
    bool mShowQuit;
    bool mShowText;
    bool mCanRunWithoutMagicPad;
    bool mShowInformationButton;

    QPixmap _bg;

};

#endif // MAINWINDOW_H
