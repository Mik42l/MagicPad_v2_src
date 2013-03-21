#ifndef NEWPROJECT_H
#define NEWPROJECT_H

#include "applet.h"

#include <QPainter>
#include <QLCDNumber>

#include "filters/calibrationFilter.h"

#define NEWPROJECT_NAME "newproject"
#define NEWPROJECT_TITLE "New Project"


class NewprojectApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    NewprojectApplet(QWidget *parent = 0);

    void setFrame( cv::Mat& frame );

signals:
  void num_object(int num);


public slots:

    void paintEvent(QPaintEvent *);

private:
    int  zoom;
    CalibrationFilter *mCalibrationFilter;

    QLCDNumber *lcd;

    //Input Matrix
    cv::Mat mFrame;

    //Pre-Processing Matrix
    cv::Mat zoomFrame;
    cv::Mat closeFrame;


    //Kalman Filter
    cv::KalmanFilter KF;
    cv::Mat_<float> state;
    cv::Mat processNoise;
    cv::Mat_<float> measurement;

    //Extracted Datas
    cv::vector<cv::Point2i> cog;
    int state_rec;

    //cv::Mat resultFrame;
    //cv::Mat outFrame;
};

#endif // NEWPROJECT_H
