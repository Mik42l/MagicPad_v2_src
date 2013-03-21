#ifndef SLIDESHOWAPPLET_H
#define SLIDESHOWAPPLET_H

#include <windows.h>
#include <WinAble.h>

#include "applet.h"

#include <QDesktopServices>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QUrl>

#include "filters/calibrationFilter.h"
#include "filters/swapFilter.h"

#include "widgets/arrowhintWidget.h"

#define SLIDESHOW_NAME "slideshow"
#define SLIDESHOW_TITLE "Slideshow"

/**
 * SlideshowApplet: navigates in a PDF or Powerpoint file.
 */
class SlideshowApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    SlideshowApplet(QWidget *parent = 0);

public slots:

    // Handles --> (left to right) movements.
    void leftToRight();

    // Open a new file.
    void openFile();

    // Handles <-- (right to left) movements.
    void rightToLeft();

    // Set new input data.
    void setFrame( cv::Mat& frame );

private:

    // Emulate a keyboard press event.
    void sendCommand( int character, int keyaction = 0 );

private:

    // Filters
    CalibrationFilter *mCalibrationFilter;
    SwapFilter *mSwapFilter;

    // Widgets
    ArrowHintWidget *mArrowWidget;


};

#endif // SLIDESHOWAPPLET_H
