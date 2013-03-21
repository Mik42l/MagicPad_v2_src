#include "joystickFilter.h"

const QString JoystickFilter::TAG = QString("JoystickFilter");

/**
 *
 */
JoystickFilter::JoystickFilter(QObject *parent)
    : Filter(parent)
{
    reset();
}

/**
 *
 */
void JoystickFilter::process()
{
    cv::Point2f bary(0, 0);

    // find threshold
    cv::Mat bw;
    double thres = cv::threshold(Frame, bw, -1, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    int rowFlag = 3;

    // barycenter
    const double sz_2 = (Frame.rows-1) * 0.5; // Half-size of the sensor
    int W = 1; // Total pixel weight (default is 1 to prevent divide-by-zero)
    int np = 0; // number of pixels contributing to the barycenter

    for(int i=0; i<Frame.rows; i++)
    {
        bool emptyRow = true;
        for(int j=0; j<Frame.cols; j++)
        {
            unsigned char w = Frame.at<unsigned char>(i, j);

            //
            // Top Otsu Barycenter
            //
            if(rowFlag>=0 && w < thres)
            {
                if(emptyRow == true)
                {
                    emptyRow = false;
                    rowFlag--;
                }
                bary += (255-w) * cv::Point2f(j-sz_2, i-sz_2) * (1.0 / sz_2);
                W += (255-w);
                np++;
            }
        }
    }

    // Normalize barycenter
    bary = bary * (1.0 / W);

    // Average value of contributing pixels
    //if(np>0) m_average = 255 - W/np;
    //else m_average = 128;

    Position = bary;
}

/**
 *
 */
void JoystickFilter::reset()
{
    Position = cv::Point2f(0.0, 0.0);
}
