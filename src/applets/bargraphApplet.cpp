#include "bargraphApplet.h"

#include <QVBoxLayout>

const QString BargraphApplet::TAG = QString("BargraphApplet");

/**
 *
 */
BargraphApplet::BargraphApplet(QWidget *parent) : Applet(parent)
{
    mName = BARGRAPH_NAME;
    mTitle = BARGRAPH_TITLE;

    // Gestures
    setGestures(SWAP_ALL);

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Pipeline
    mCalibrationFilter = new CalibrationFilter( this );

    mFrame = cv::Mat( 10, 10, CV_8U );

    // User interface
    setLayout( new QVBoxLayout(this) );

    /*** ***/
    mAlpha = 45.0;          // 45
    mBeta = 30.0;           // 30
    mScaleFactor = 1.0;
    mBarWidth = 20;
    mBarSpacing = 40;
    mBarSpacing_y = 20;
    // Set default size for graphic drawing
    mBarWidth = mBarWidth * mScaleFactor;
    mBarSpacing = mBarSpacing * mScaleFactor * 1.3;
    mBarSpacing_y = mBarSpacing_y * mScaleFactor;
    mBarMargin = 2 * mBarWidth;
    //Issue to calculate the height of the window
    mBarWindowH = 2 * (mBarMargin + mBarSpacing) + 255;
    mBarWindowW = 2 * mBarMargin + (mFrame.cols*mBarWidth+(mFrame.cols-1)*mBarSpacing)*(90.0-mAlpha)/90.0 + (mFrame.rows*mBarWidth+(mFrame.rows-1)*mBarSpacing_y)*(90-mBeta)/90;
    mOrigin = 0;

    //setFixedSize(BarWindowW, BarWindowH+(numberofsensors_col*BarWidth+(numberofsensors_col-1)*BarSpacing)*(90.0-alpha)/90.0);
}

/**
 *
 */
void BargraphApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    mFrame = mCalibrationFilter->getCalibratedFrame();

    update();
}

/**
 *
 */
void BargraphApplet::drawBar(QPainter *painter, int x, int y, int l, int h)
{
    const QColor edge( 100, 40, 32 );
    const QColor light( 255, 83, 53 );
    const QColor dark( 166 ,55, 35 );

    //const QColor top( 200, 70, 43 );
    // top color is a linear interpolation between 'dark' and 'light' edge
    QColor top( light.red() - (light.red()-dark.red())/255.0*h,
                light.green() - (light.green()-dark.green())/255.0*h,
                light.blue() - (light.blue()-dark.blue())/255.0*h );

    float hc; // calculate local heigth depending on angle
    hc = h*(90-mBeta)/90;

    // Point p1
    float x1, y1;
    x1 = x+l*(90-mAlpha)/90;
    if(mAlpha>0)
    {
        y1 = y+l*mBeta/90;
    }
    else
    {
        y1 = y;
    }

    // Point p2
    float x2, y2;
    x2 = x+l;
    if(mAlpha>0)
    {
        y2 = y;
    }
    else
    {
        y2 = y-l*mBeta/90;
    }

    // Point p3
    float x3, y3;
    x3 = x+(x2-x1);
    y3 = y-hc-l*mBeta/90;

    // Left side
    QPointF leftSide[4] = {
         QPointF(x, y),
         QPointF(x1, y1),
         QPointF(x1, y1-hc),
         QPointF(x, y-hc)
    };

    // Right side, slightly darker
    QPointF rightSide[4] = {
         QPointF(x1, y1),
         QPointF(x2, y2),
         QPointF(x2, y2-hc),
         QPointF(x1, y1-hc)
    };

    // Top side
    QPointF topSide[4] = {
         QPointF(x1, y1-hc),
         QPointF(x2, y2-hc),
         QPointF(x3, y3),
         QPointF(x, y-hc)
    };

    // Draw shadow
    painter->translate( 3, -2 );
    painter->setPen( Qt::NoPen );
    painter->setBrush( QColor(0, 0, 0, 75) );
    painter->drawPolygon(leftSide, 4);
    painter->drawPolygon(rightSide, 4);
    painter->drawPolygon(topSide, 4);
    painter->translate( -3, 2 );

    // Draw Bar
    painter->setPen( Qt::SolidLine );
    painter->setPen( edge );
    painter->setBrush( light );
    painter->drawPolygon(leftSide, 4);
    painter->setBrush( dark );
    painter->drawPolygon(rightSide, 4);
    painter->setBrush( top );
    painter->drawPolygon(topSide, 4);
}

/**
 *
 */
void BargraphApplet::paintEvent(QPaintEvent *)
{
    QPainter painter(this);

    painter.setRenderHint( QPainter::Antialiasing );

    /// Draw background (if any)
    painter.fillRect( rect(), QColor(0, 0, 0, 70) );

    // Set viewport to a square area centered
    int sz = qMin( rect().width(), rect().height() );
    QRect vp(0, 0, sz, sz);
    vp.moveCenter( rect().center() );
    painter.setViewport( vp );
    ///painter.setBrush( Qt::blue );
    ///painter.drawRect( vp );

    painter.setWindow(0, 0, mBarWindowW, mBarWindowH+(mFrame.cols*mBarWidth+(mFrame.cols-1)*mBarSpacing)*(90.0-mAlpha)/90.0 );

    // Draw Bar
    int value;
    int offset = 30;

    // Preparation for label over the bar
    int x, y = 0;

    for(int j=mFrame.rows-1; j>-1; j--)
    {
        for(int i=0; i<mFrame.cols; i++)
        {
            value = mFrame.at<unsigned char>(i,j);
            x = mOrigin + mBarMargin + (mBarWidth + mBarSpacing)*(90 - mAlpha) / 90 * i + (mBarWidth + mBarSpacing_y)*(j - 0)*(90 - mBeta)/90;
            y = mBarWindowH - mBarMargin + (mBarWidth + mBarSpacing)*(90 - mAlpha) / 90 * i - (mBarWidth + mBarSpacing_y)*(j - 0)*(90 - mBeta)/90 + offset;
            drawBar(&painter, x, y, mBarWidth, value);
        }
    }
}
