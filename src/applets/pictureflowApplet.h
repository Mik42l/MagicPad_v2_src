#ifndef PICTUREFLOWAPPLET_H
#define PICTUREFLOWAPPLET_H

#include "applet.h"

#include <QApplication>
#include <QDir>
#include <QPainter>
#include <QResizeEvent>

#include "filters/calibrationFilter.h"
#include "filters/swapFilter.h"

#include "widgets/pictureflow.h"

#define PICTUREFLOW_NAME "pictureflow"
#define PICTUREFLOW_TITLE "Picture Flow"

#define PICTUREFLOW_IMAGES_PATH "/data/pictureflow"

class PictureFlowApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    PictureFlowApplet(QWidget *parent = 0);

public slots:

    void paintEvent(QPaintEvent *) { }

    void setFrame( cv::Mat& frame );

private:

    QStringList findFiles( const QString &path = QString() );

    void resizeEvent(QResizeEvent *event)
    {
        if(mPictureFlow) mPictureFlow->resize( event->size() );
    }

private:

    CalibrationFilter *mCalibrationFilter;

    SwapFilter *mSwapFilter;

    PictureFlow *mPictureFlow;
};

#endif // PICTUREFLOWAPPLET_H
