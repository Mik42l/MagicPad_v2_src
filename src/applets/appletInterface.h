#ifndef APPLETINTERFACE_H
#define APPLETINTERFACE_H

#include <QWidget>
#include <opencv2/opencv.hpp>

/**
 * This class describes the common interface to any Applet.
 * Native applet as well as Plugin applet should implement this interface.
 *
 * For more information about Plugins and Interfaces, please check out:
 * - http://qt-project.org/doc/qt-4.8/plugins-howto.html
 */

#define APPLET_INTERFACE_GESTURE_NUMBER   3

class AppletInterface
{


public:

    // Type of application gesture
    typedef enum {
        NONE            = 0,
        TWIST,
        SWAP_LEFT_RIGHT,
        SWAP_BACK_FORTH,
        SWAP_ALL,
        UP_DOWN,
        TOP_RIGHT,
        BOTTOM_RIGHT,
        ALPHABET
        } GestureType;


    virtual ~AppletInterface() {}

    // Return the applet short name ( lower char, no spaces )
    // The applet name is used for the icon path, applet data, ...
    virtual QString name() = 0;

    // Return the full applet title ( mixed case, spaces ) that appears
    // bellow the applet icon on the main screen
    virtual QString title() = 0;

    // Return the applet icon ( use $SOURCES/artwork/icon as a guideline)
    virtual QPixmap icon() = 0;

    // Return the accepted gestures
    virtual void acceptedGestures(QPixmap * ges) = 0;

    // Set the accepte gestures, to use in constructor
    virtual void setGestures(GestureType ges0, GestureType ges1 = NONE, GestureType ges2 = NONE) = 0;

    // Every applet has three paragraphs that:
    // - describes what the applet does and how to use it
    // - why is it better than existing solution
    // - gives some technical insight of the technology, limitations (if any ...), ...
    virtual QString descriptionText() = 0;
    virtual QString marketingText() = 0;
    virtual QString technicalText() = 0;

    // This slot is call every time a new frame is received from the magicpad
    // If the applet is not visible/in use, then this is not called
    virtual void setFrame( cv::Mat& frame ) = 0;

    // Serves the same purpose as with QWidgets
    virtual void hide() = 0;

    // Serves the same purpose as with QWidgets
    virtual void resize( const QSize& size ) = 0;

    // Serves the same purpose as with QWidgets
    virtual void show() = 0;

    // Called when an applet is launched
    // Can be used to reset some values/filters/...
    virtual void start() = 0;

    // Called when the user go back to the main menu
    virtual void stop() = 0;

    // Common method implementation is:
    //    return qobject_cast<QWidget *>(this);
    virtual void setParent(QWidget *parent) = 0;

};

Q_DECLARE_INTERFACE(AppletInterface, "AppletInterface")

#endif // APPLETINTERFACE_H
