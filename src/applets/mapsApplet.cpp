#include "mapsApplet.h"

const QString MapsApplet::TAG = QString("MapsApplet");

/**
 *
 */
MapsApplet::MapsApplet(QWidget *parent) : Applet(parent)
{
    mName = MAPS_NAME;
    mTitle = MAPS_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter( this );
    mJoystickFilter = new JoystickFilter( this );
    mObjectDetectionFilter = new ObjectDetectionFilter();
    reset();

    // Build user interface
    mWebView = new QWebView( this );
    mWebView->settings()->setAttribute( QWebSettings::JavascriptEnabled, true );
    mWebView->load( QUrl::fromLocalFile( MAPS_HTML_FILE ) );

    QHBoxLayout *layout = new QHBoxLayout( this );
    layout->addWidget( mWebView );

    // Gestures
    setGestures(SWAP_ALL);
}

/**
 *
 */
void MapsApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    mJoystickFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mJoystickFilter->process();

    cv::Point2f p(0, 0);
    if(mObjectDetectionFilter->getObjectDetected() == true)
    {
        // Update position + low pass filter
        p = mJoystickFilter->getPosition();
        mPos += (p-mPos)*0.80;
    }
    else
    {
        // when no object is detected, slowly set the joystick to the central position
        mPos *= 0.7;
    }

    // Send javascript cmd to move the map
    mWebView->page()->mainFrame()->evaluateJavaScript( QString("panMap(%1, %2)").arg(mPos.x * MAPS_SPEED).arg(mPos.y * MAPS_SPEED) );

    update();
}

/**
 *
 */
void MapsApplet::reset()
{
    mCalibrationFilter->reset();
    mJoystickFilter->reset();
    mObjectDetectionFilter->reset();

    mPos.x = 0;
    mPos.y = 0;
}

/**
 *
 */
void MapsApplet::start() {
    if( !connexionOk() )
    {
        QMessageBox::warning(this, "No internet connexion", "The 'maps' applet requires Internet access.");
    }

    Applet::start();
}

/**
 * Check the internet connexion
 */
bool MapsApplet::connexionOk() {
    QTcpSocket *socket = new QTcpSocket;
    socket->connectToHost( "www.google.com", 80 );
    return socket->waitForConnected( 3000 );
}

