#include "opticalflowWidget.h"

const QString OpticalFlowWidget::TAG = QString("OpticalFlowWidget");

/**
 *
 */
OpticalFlowWidget::OpticalFlowWidget(QWidget *parent) :
    QWidget(parent)
{
    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::transparent);
    this->setPalette(p);
}

/**
 *
 */
void OpticalFlowWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    //painter.fillRect( rect(), Qt::red );

    painter.setRenderHint(QPainter::Antialiasing, true);
    int side = qMin(width(), height());
    painter.setViewport((width() - side) / 2, (height() - side) / 2,
                        side, side);
    painter.setWindow(0, 0, mFlow.cols*10, mFlow.rows*10);

    //
    // Paint image
    //

    painter.setPen( Qt::NoPen );
    for(int r=0; r<mFlow.rows; ++r)
    {
        for(int c=0; c<mFlow.cols; ++c)
        {
            // Draw rect
            unsigned char value = mFrame.at<unsigned char>(r, c);
            painter.fillRect(c*10, r*10, 11, 11, QColor(value, value, value) );
        }
    }

    //
    const float vectorSize = 15.0;
    QPen v1pen( Qt::black );
    v1pen.setWidthF( 0.9 );
    v1pen.setCapStyle( Qt::RoundCap );

    QPen v2pen( Qt::green );
    v2pen.setWidthF( 0.8 );
    v2pen.setCapStyle( Qt::RoundCap );


    for(int r=0; r<mFlow.rows; ++r)
    {
        for(int c=0; c<mFlow.cols; ++c)
        {
            // pel center point
            QPointF center(c*10 + 5, r*10 + 5);

            // vector
            QPointF delta = center + vectorSize*QPointF(
                mFlow.at<cv::Point2f>(r,c).x,
                mFlow.at<cv::Point2f>(r,c).y);

            // draw outline
            painter.setPen( v1pen );
            painter.drawLine( center, delta );

            painter.setPen( v2pen );
            painter.drawLine( center, delta );
        }
    }
}
