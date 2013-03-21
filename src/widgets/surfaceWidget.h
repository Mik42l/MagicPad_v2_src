#ifndef SURFACEWIDGET_H
#define SURFACEWIDGET_H

#include <opencv2/opencv.hpp>
#include "qwt3d_surfaceplot.h"

#define SURFACE_PLOTFACTOR 20 // Maximum of steps on z-axis for 3D surface plot

/**
 *
 */
class SurfaceWidget : public Qwt3D::SurfacePlot
{

    Q_OBJECT

public:

    static const QString TAG;

    SurfaceWidget(QWidget *parent = 0);

public slots:
    void setFrame( const cv::Mat& frame );

private:
   void allocateData();
   void setUpscale(int newRes);

private:
   // Raw frame from sensor
   cv::Mat m_frameFiltered;
   cv::Mat m_frameDouble;

   // High resolution frame
   cv::Mat m_highResFrame;

   // Resolution factor: m_highResFrame is m_resolution times
   // bigger than m_frame
   int m_resolution;
   int m_hrWidth;
   int m_hrHeight;

   // local buffer
   double **m_tabla;

   // color vector: store 100 RGB color ranging from dark blue to bright red
   Qwt3D::ColorVector m_cv;

};

#endif // SURFACEWIDGET_H
