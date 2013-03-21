#include "newprojectApplet.h"

#include <QVBoxLayout>


const QString NewprojectApplet::TAG = QString("NewprojectApplet");

/**
 *
 */
NewprojectApplet::NewprojectApplet(QWidget *parent) : Applet(parent)
{
    mName = NEWPROJECT_NAME;
    mTitle = NEWPROJECT_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    zoom = 10;
    state_rec = 0;

    // Filters pipeline
    mCalibrationFilter = new CalibrationFilter();

    mFrame = cv::Mat();


    zoomFrame =  cv::Mat(mFrame.rows*zoom,mFrame.cols*zoom,CV_8UC1);
    closeFrame = cv::Mat(zoomFrame.rows,zoomFrame.cols,CV_8UC1);

    KF = cv::KalmanFilter(4, 2, 0);
    state =cv::Mat_<float> (4, 1); /* (x, y, Vx, Vy) */
    //cv::Mat processNoise(4, 1, CV_32F);
    measurement = cv::Mat_<float> (2,1); measurement.setTo(cv::Scalar(0));
    KF.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,3,0,   0,1,0,3,  0,0,1,0,  0,0,0,1);

    cog = cv::vector<cv::Point2i>(2);

    //outFrame = cv::Mat(zoomFrame.rows,zoomFrame.cols,CV_8UC1);

    //User Interface
    QVBoxLayout *layout = new QVBoxLayout( this );
    lcd = new QLCDNumber( this );
    layout->addWidget( lcd );
    connect(this,SIGNAL(num_object(int)),lcd,SLOT(display(int)));
    // Gestures
    setGestures(SWAP_ALL);
}

/**
 * paintEvent - paint the content of the widget
 */
void NewprojectApplet::paintEvent(QPaintEvent *)
{

    if( mFrame.empty() ) {
        return;
    }
    cv::Mat outFrame = cv::Mat::zeros(zoomFrame.rows,zoomFrame.cols,CV_8UC1);
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    cv::Mat resultFrame = cv::Mat::zeros(zoomFrame.rows,zoomFrame.cols,CV_8UC1);

    cv::findContours(closeFrame,contours,hierarchy, CV_RETR_CCOMP , CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    cv::vector<cv::Moments> mu(contours.size() );
    cv::vector<cv::Point2i> mc( contours.size() );
    cv::vector<cv::RotatedRect> ellv(contours.size());
    cv::vector<cv::vector<cv::Point> > hull( contours.size() );
    cv::vector<cv::Point> kalmanv;


    if (contours.size() != 0){
        for( int i = 0; i < contours.size(); i++ ){

            mu[i] = cv::moments( contours[i], false );
            mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );

            cv::Scalar color( i*30+50, i*30+10,i*30+100  );
            cv::convexHull( cv::Mat(contours[i]), hull[i], false);
            //cv::convexHull( cv::Mat(contours[i]), hull_points[i]);

            //cv::vector<cv::Vec4i>  convexityDefects;

            //        if (hull[i].size() > 2){
            //        //cv::convexityDefects(contours[i],hull[i],convexityDefects);
            //drawContours( zoomFrame, hull[i], i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
            //       // QLOG_DEBUG() << TAG << hull[i].size() << convexityDefects.size();
            //        }


            //cv::drawContours(outFrame, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
            //            if (mu[i].m00 >= 10){

            if (contours[i].size() > 5){
                ellv[i] = fitEllipse(contours[i]);



//                //Kalman's Filter

//                KF.statePre.at<float>(0) = mc[i].x;
//                KF.statePre.at<float>(1) = mc[i].y;
//                KF.statePre.at<float>(2) = 0;
//                KF.statePre.at<float>(3) = 0;


//                setIdentity(KF.measurementMatrix);
//                setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-4));
//                setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
//                setIdentity(KF.errorCovPost, cv::Scalar::all(.1));


//                kalmanv.clear();
//                //cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));


//                //            Point statePt(state(0),state(1));

//                cv::Mat prediction = KF.predict();
//                cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

//                measurement(0) = mc[i].x;
//                measurement(1) = mc[i].y;

//                cv::Point measPt(measurement(0),measurement(1));

//                //generate measurement
//                //measurement += KF.measurementMatrix*state;

//                cv::Mat estimated = KF.correct(measurement);
//                cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
//                kalmanv.push_back(statePt);


//                // drawCross( statePt, Scalar(255,255,255), 5 );
//                // drawCross( measPt, Scalar(0,0,255), 5 );
//                cog[i] = statePt;
//                ellv[i].center = statePt;

                if (contours.size() == 1){
                    if (mu[0].m00 < 3200){
                        state_rec = 1;
                        ellv[0].size = cv::Size2f(10,30);
                        //cv::ellipse(resultFrame,ellv[0],color);
                    }
                    else{
                        state_rec = 3;
                        ellv[0].size = cv::Size2f(40,40);
                        //cv::ellipse(resultFrame,ellv[0],color);
                    }
                }

                else if ((mu[1].m00 > 800) && (mu[0].m00 > 800)){
                    state_rec = 2;
                    ellv[0].size = cv::Size2f(10,30);
                    ellv[1].size = cv::Size2f(10,30);
                    ellv[2].size = cv::Size2f(10,30);
                    //cv::ellipse(resultFrame,ellv[0],color);
                    //cv::ellipse(resultFrame,ellv[1],color);
                    //cv::ellipse(resultFrame,ellv[2],color);
                }

                //QLOG_DEBUG() << TAG << i << mu[i].m00;
                //QLOG_DEBUG() << TAG << i << cog[i].x << cog[i].y ;

                cv::Point2i mPoint = cv::Point2i((int)cog[i].x/10,(int)cog[i].y/10);

                unsigned char col = ( mFrame.at<unsigned char>(mPoint)
                                        + mFrame.at<unsigned char>(cv::Point2i(mPoint.x + 1,mPoint.y))
                                        + mFrame.at<unsigned char>(cv::Point2i(mPoint.x - 1,mPoint.y))
                                        + mFrame.at<unsigned char>(cv::Point2i(mPoint.x ,mPoint.y + 1))
                                        + mFrame.at<unsigned char>(cv::Point2i(mPoint.x ,mPoint.y - 1)) );

//                col = (int)col/5;
                col = 255 - col;
                QLOG_DEBUG() << TAG << i << col ;
                cv::Scalar color( col, col,col  );

                if (contours.size() <= 2){
                    //cv::ellipse(resultFrame,ellv[i],color);
                    //cv::circle(resultFrame,mc[i],2,color);
                    cv::circle(resultFrame,mc[i],5,color);
                    //cv::circle(resultFrame,measPt,2,color);
                    //cv::circle(resultFrame,predictPt,8,color);
                }
                //        int hull_size = hull[i].size();
                //        cv::Point2i middle = cv::Point2i(

                //                    hull[i][(int)hull_size/2 - 2] + hull[i][(int)hull_size/2 - 1] + hull[i][(int)hull_size/2] + hull[i][(int)hull_size/2 + 1]+ hull[i][(int)hull_size/2 + 2]
                //                    );
                //        middle.x = middle.x/5;
                //        middle.y = middle.y/5;
                //        cv::circle(zoomFrame,middle,4,color);
                //cv::circle(resultFrame,mc[i],2,color);
                //        for (int j = 0; j<hull[i].size(); j++){
                //            cv::circle(zoomFrame, middle, 2, color, -1, 8, 0 );
                //            //QLOG_DEBUG() << TAG << j <<convexityDefects[j][1];
                //        }

                //resultFrame.at<unsigned char>(mc[i].x,mc[i].y)=255;


            }
            // mean = cv::mean(mFrame);
        }
    }
    else state_rec = 0;

    emit(num_object(state_rec));

    QPainter painter( this );

    // Placeholder drawing
    painter.setPen( Qt::red );
    painter.setBrush( Qt::black );
    painter.fillRect( rect(), Qt::black );

    // draw image
    painter.setPen( Qt::transparent );
    double w = rect().width() / zoomFrame.cols;
    double h = rect().height() / zoomFrame.rows;

    for(int i=0; i<zoomFrame.rows; ++i)
    {
        for( int j=0; j<zoomFrame.cols; ++j )
        {
            QRect r( j*w, i*h, w, h );

            unsigned  char c =  resultFrame.at<unsigned char>(i,j);
            painter.setBrush(QColor(c,c,c));
            painter.drawRect( r );
            //painter.drawText( r, "  " + QString::number( c ) );
        }
    }

}
/**
 *
 */
void NewprojectApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    mFrame = mCalibrationFilter->getCalibratedFrame();


    cv::Scalar mean = cv::Scalar();
    // Preprocessing : track object using moments & contours

    // Resize Image mthFrame -> zoomFrame
    //cv::medianBlur(mFrame,mFrame,3);
    cv::resize(mFrame,zoomFrame,zoomFrame.size(),zoom,zoom,cv::INTER_CUBIC  );

    cv::medianBlur(zoomFrame,zoomFrame,19);

//    for (int i=0; i<zoomFrame.rows; i++){
//        for (int j=0; j<zoomFrame.cols; j++){

//            if (zoomFrame.at<unsigned char>(i,j) > 180 ) zoomFrame.at<unsigned char>(i,j) = 0;
//            else zoomFrame.at<unsigned char>(i,j) = 255;
//        }
//    }

    cv::adaptiveThreshold(zoomFrame, zoomFrame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV , 15, 0);

    //TO DO : OPENING
    //Morphological operations : Remove Small Objects zoomFrame->openFrame

    cv::Mat struct_element = cv::Mat();

    struct_element= cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10) );
    morphologyEx(zoomFrame,closeFrame,cv::MORPH_OPEN,struct_element);


    cv::Mat struct_element_2 = cv::Mat();

    struct_element_2= cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,30) );
    morphologyEx(closeFrame,closeFrame,cv::MORPH_CLOSE,struct_element_2);




    //emit(num_object(contours.size()));
    update();
}
