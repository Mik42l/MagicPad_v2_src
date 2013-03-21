#ifndef OPTICALFLOWWIDGET_H
#define OPTICALFLOWWIDGET_H

#include <QWidget>
#include <QPainter>

#include <opencv2/opencv.hpp>

/**
 *
 */
class OpticalFlowWidget : public QWidget
{
    Q_OBJECT
	
public:

    static const QString TAG;

    OpticalFlowWidget(QWidget *parent = 0);
	
protected:

    void paintEvent(QPaintEvent *event);

public slots:

    void setFlow( const cv::Mat& flow ) {
        mFlow = flow;
    }

    void setFrame( const cv::Mat& frame ) {
        mFrame = frame;
    }

private:

    cv::Mat mFlow;
    cv::Mat mFrame;

};

#endif // OPTICALFLOWWIDGET_H
