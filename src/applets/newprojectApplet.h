#ifndef NEWPROJECT_H
#define NEWPROJECT_H

#include "applet.h"

#include <QPainter>
#include <QLCDNumber>

#include "filters/calibrationFilter.h"

#define NEWPROJECT_NAME "newproject"
#define NEWPROJECT_TITLE "New Project"

#define ZOOM 10
#define MAX_POINT_MEMORY 20


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

    CalibrationFilter *mCalibrationFilter;

    QLCDNumber *lcd;

    //Input Matrix
    cv::Mat mFrame;

    //Pre-Processing Matrix
    cv::Mat zoomFrame;
    cv::Mat closeFrame;
    //cv::Mat drawFrame;

    //Kalman Filter
    cv::KalmanFilter KF;
    cv::Mat_<float> state;
    cv::Mat processNoise;
    cv::Mat_<float> measurement;


    //Extracted Datas
    //Number of objects detected in the imput
    int nb_objects;
    //Contour of each object
    cv::vector<cv::vector<cv::Point> > contours;
    //Hierarchy of the contours
    cv::vector<cv::Vec4i> hierarchy;
    //Matrix containing the MAX_POINT_MEMORY last positions of the cog of each object
    cv::Mat3i cog;
    cv::Mat3i cogvel;
    //TO DO
    int state_rec;
    //Index of the frame. We keep MAX_POINT_MEMORY data.
    int frame_idx;

    //cv::Mat resultFrame;
    //cv::Mat outFrame;
};

#endif // NEWPROJECT_H
