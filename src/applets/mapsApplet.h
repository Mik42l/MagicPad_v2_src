#ifndef MAPSAPPLET_H
#define MAPSAPPLET_H

#include "applet.h"

#include <QtWebKit/QtWebKit>
#include <QHBoxLayout>
#include <QMessageBox>

#include "filters/calibrationFilter.h"
#include "filters/joystickFilter.h"
#include "filters/objectDetectionFilter.h"

#define MAPS_NAME "maps"
#define MAPS_TITLE "Maps"

// Speed factor
#define MAPS_SPEED 1.5

// HTML file
#define MAPS_HTML_FILE (qApp->applicationDirPath() + "/data/maps/maps.html")

/**
 *
 */
class MapsApplet : public Applet
{
    Q_OBJECT
	
public:

    static const QString TAG;

    MapsApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void start();

    void reset();

private:

    bool connexionOk();

private:
    CalibrationFilter *mCalibrationFilter;
    JoystickFilter *mJoystickFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;
    cv::Point2f mPos;
    QWebView *mWebView;
};

#endif // MAPSAPPLET_H
