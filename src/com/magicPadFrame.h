#ifndef MAGICPADFRAME_H
#define MAGICPADFRAME_H

#include <opencv2/opencv.hpp>

#include <QtCore>

/**
 *
 */
class MagicPadFrame
{
public:
    MagicPadFrame()
    {
        img = cv::Mat(10, 10, CV_8U);
        timestamp = 0;
    }

    MagicPadFrame(unsigned char *data, long unsigned int _timestamp = 0)
    {
        img = cv::Mat(10, 10, CV_8U, data);
        timestamp = _timestamp;
    }

    MagicPadFrame(cv::Mat& _img, long unsigned int _timestamp = 0)
    {
        img = _img;
        timestamp = _timestamp;
    }

    cv::Mat img;
    long unsigned int timestamp;
};

#endif // MAGICPADFRAME_H
