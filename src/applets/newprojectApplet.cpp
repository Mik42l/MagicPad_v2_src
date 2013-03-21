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

    state_rec = 0;
    nb_objects = 0;
    frame_idx = 0;

    // Filters pipeline
    mCalibrationFilter = new CalibrationFilter();
    //Input Frame
    mFrame = cv::Mat();

    //Preprocessing Matri
    zoomFrame =  cv::Mat(mFrame.rows*ZOOM,mFrame.cols*ZOOM,CV_8UC1);
    closeFrame = cv::Mat(zoomFrame.rows,zoomFrame.cols,CV_8UC1);
    //skel = cv::Mat::zeros(zoomFrame.size(), CV_8UC1);
    //drawFrame = cv::Mat(zoomFrame.rows,zoomFrame.cols,CV_8UC1);


    KF = cv::KalmanFilter(8, 4, 0);
    state =cv::Mat_<float> (8, 1); /* (x, y, z, alpha, Vx, Vy, Vz, Valpha) */
    //cv::Mat processNoise(4, 1, CV_32F);
    measurement = cv::Mat_<float> (4,1); measurement.setTo(cv::Scalar(0));
    KF.transitionMatrix = *(cv::Mat_<float>(8, 8) << 1,0,0,0,3,0,0,0,
                            0,1,0,0,0,3,0,0,
                            0,0,1,0,0,0,3,0,
                            0,0,0,1,0,0,0,3,
                            0,0,0,0,1,0,0,0,
                            0,0,0,0,0,1,0,0,
                            0,0,0,0,0,0,1,0,
                            0,0,0,0,0,0,0,1
                            );

    KF.statePre.at<float>(0) = 0;
    KF.statePre.at<float>(1) = 0;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    KF.statePre.at<float>(4) = 0;
    KF.statePre.at<float>(5) = 0;



    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1*1e-4));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1*1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1*.1));

    contours = cv::vector<cv::vector<cv::Point> > ();
    hierarchy = cv::vector<cv::Vec4i>();
    cog = cv::Mat3i::zeros(2,MAX_POINT_MEMORY) ;
    cogvel = cv::Mat3i::zeros(2,MAX_POINT_MEMORY) ;
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

    //    cv::vector<cv::vector<cv::Point> > contours;
    //    cv::vector<cv::Vec4i> hierarchy;

    cv::Mat resultFrame = cv::Mat::zeros(zoomFrame.rows,zoomFrame.cols,CV_8UC1);

    //Find objects based on their contours
    //contours : the vector of points for each contours
    //hierarcgy : the hierarchical strucutre of the objects.
    cv::findContours(closeFrame,contours,hierarchy, CV_RETR_CCOMP , CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    cv::vector<cv::Moments> mu(contours.size() );
    cv::vector<cv::Point2i> mc( contours.size() );
    cv::vector<cv::RotatedRect> ellv(contours.size());
    cv::vector<cv::vector<cv::Point> > hull( contours.size() );
    cv::vector<cv::Point> kalmanv;
    cv::vector<cv::Point2i> middle(contours.size());
    nb_objects = contours.size();

    emit (num_object(nb_objects));


    //For each object
    if (nb_objects != 0){
        QLOG_DEBUG() << "New Frame" ;
        for( int i = 0; i < nb_objects; i++ ){

            //Compure moments and extract cog
            mu[i] = cv::moments( contours[i], false );
            mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
            //cog[i][frame_idx][0] = mc[i].x; cog[i][frame_idx][1] = mc[i].y;


            middle[i].x = (contours[i][0].x + contours[i][contours[i].size()-1].x)/2;
            middle[i].y = (contours[i][0].y + contours[i][contours[i].size()-1].y)/2;

            //cv::circle(closeFrame,middle[i],2,cv::Scalar(255,255,255));
//            cv::circle(closeFrame,contours[i][0],2,cv::Scalar(255,255,255));
//            cv::circle(closeFrame,contours[i][contours[i].size()-1],2,cv::Scalar(255,255,255));

//            for (int j = 0; j < contours[i].size();j++){
//                //cv::circle(closeFrame,contours[i][j],1,cv::Scalar(255,255,255));
//                cv::line(closeFrame,contours[i][j],contours[i][(j+1+contours[i].size())%contours[i].size()],cv::Scalar(255,255,255));
//            }

            cv::Scalar color( i*30+50, i*30+10,i*30+100  );

            //Extract the convexhull of the object
            cv::convexHull( cv::Mat(contours[i]), hull[i], false);
            //cv::convexHull( cv::Mat(contours[i]), hull_points[i]);

            //cv::vector<cv::Vec4i>  convexityDefects;

            //        if (hull[i].size() > 2){
            //        //cv::convexityDefects(contours[i],hull[i],convexityDefects);
            //cv::drawContours( closeFrame, contours, i, color, 1, 8, cv::vector<cv::Vec4i>(), 0, cv::Point() );
            //       // QLOG_DEBUG() << TAG << hull[i].size() << convexityDefects.size();
            //        }


            // cv::drawContours(resultFrame, hull, i, color, 1, 8, hierarchy, 0, cv::Point() );
            //            if (mu[i].m00 >= 10){

            //For significant objects (ie area > 5 px)
            if (contours[i].size() > 5){
                //Find the best fitting ellipse
                ellv[i] = cv::minAreaRect(contours[i]);


                cv::Point2i mPoint = cv::Point2i((int)cog[i][frame_idx][0]/10,(int)cog[i][frame_idx][1]/10);




                //                int hull_size = hull[i].size();
                //                cv::Point2i middle = cv::Point2i(

                //                            hull[i][(int)hull_size/2 - 2] + hull[i][(int)hull_size/2 - 1] + hull[i][(int)hull_size/2] + hull[i][(int)hull_size/2 + 1]+ hull[i][(int)hull_size/2 + 2]
                //                            );
                //                middle.x = middle.x/5;
                //                middle.y = middle.y/5;
                //cv::circle(zoomFrame,middle,4,color);
                //cv::circle(resultFrame,mc[i],2,color);
                for (int j = 0; j<hull[i].size(); j++){
                    //cv::circle(resultFrame, middle, 2, color, -1, 8, 0 );
                    //QLOG_DEBUG() << TAG << j <<convexityDefects[j][1];
                }

                //resultFrame.at<unsigned char>(mc[i].x,mc[i].y)=255;


            }
            // mean = cv::mean(mFrame);
        }
        //                //Kalman's Filter

        //        unsigned char col = ( mFrame.at<unsigned char>(mPoint)
        //                              + mFrame.at<unsigned char>(cv::Point2i(mPoint.x + 1,mPoint.y))
        //                              + mFrame.at<unsigned char>(cv::Point2i(mPoint.x - 1,mPoint.y))
        //                              + mFrame.at<unsigned char>(cv::Point2i(mPoint.x ,mPoint.y + 1))
        //                              + mFrame.at<unsigned char>(cv::Point2i(mPoint.x ,mPoint.y - 1)) );

        //        col = (int)col/5;
        //        col = 255 - col;
        //        cog[i][frame_idx][2] = col;

        //        cv::Scalar color( col, col,col  );



        kalmanv.clear();
        //cv::randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));


        //            Point statePt(state(0),state(1));

        cv::Mat prediction = KF.predict();
        cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

        measurement(0) = middle[0].x;
        measurement(1) = middle[0].y;
        measurement(2) = 0;
        measurement(3) = ellv[0].angle;


        cv::Point measPt(measurement(0),measurement(1));

        //generate measurement
        //measurement += KF.measurementMatrix*state;

        cv::Mat estimated = KF.correct(measurement);
        cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        kalmanv.push_back(statePt);
        cv::Point2i corrPoint = cv::Point2i(statePt.x , statePt.y );

        //ellv[0].center = statePt;
        //ellv[0].angle = ((int)estimated.at<float>(3)+180) % 180;
        //ellv[0].size = cv::Size2f(10,30);
        cog[0][frame_idx][0] = statePt.x; cog[0][frame_idx][1] =statePt.y;
        cogvel[0][frame_idx][0] = ( cog[0][frame_idx][0] - cog[0][((frame_idx-1)+MAX_POINT_MEMORY)%MAX_POINT_MEMORY][0] ) / 2;
        cogvel[0][frame_idx][1] = ( cog[0][frame_idx][1] - cog[0][((frame_idx-1)+MAX_POINT_MEMORY)%MAX_POINT_MEMORY][1] ) / 2;
        //drawFrame.at<unsigned char>(stateP) = 255;

        if (contours.size() <= 2){
            //cv::ellipse(resultFrame,ellv[0],cv::Scalar( 255, 255,255  ));
        }

        cv::circle(resultFrame,statePt,2,cv::Scalar( 255, 255,255  ));
        cv::Point2i cogv = cv::Point2i(statePt.x + 5*cogvel[0][frame_idx][0],statePt.y + 5*cogvel[0][frame_idx][1]);
        //cv::line(resultFrame,statePt,cogv,cv::Scalar(150,150,150),1);
        //TODO cv::circle(resultFrame,statePt,2,cv::Scalar( 255, 255,255  ));
        //cv::circle(resultFrame,mc[0],2,cv::Scalar( 150, 150,150  ));

        frame_idx = (frame_idx+1)%MAX_POINT_MEMORY;
    }


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

            if (i == cog[0][frame_idx-1][0]  && j == cog[0][frame_idx-1][1]){
                painter.setFont( QFont("Times", 20));
                painter.drawText( r, "  " + QString::number( 1 ) );
            }

        else{


            unsigned  char c =  resultFrame.at<unsigned char>(i,j);
            painter.setBrush(QColor(c,c,c));
            painter.drawRect( r );}
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
    cv::resize(mFrame,zoomFrame,zoomFrame.size(),ZOOM,ZOOM,cv::INTER_CUBIC  );

    cv::medianBlur(zoomFrame,zoomFrame,19);

    // HANDWRITTEN THRESHOLD
    for (int i=0; i<zoomFrame.rows; i++){
        for (int j=0; j<zoomFrame.cols; j++){

            if (zoomFrame.at<unsigned char>(i,j) > 180 ) zoomFrame.at<unsigned char>(i,j) = 0;
            else zoomFrame.at<unsigned char>(i,j) = 255;
        }
    }

    //cv::adaptiveThreshold(zoomFrame, zoomFrame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV , 11, 0);


    //Morphological operations : Remove Small Objects zoomFrame->openFrame

    cv::Mat struct_element = cv::Mat();

    struct_element= cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(10,10) );
    morphologyEx(zoomFrame,closeFrame,cv::MORPH_OPEN,struct_element);


    cv::Mat struct_element_2 = cv::Mat();

    struct_element_2= cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(20,30) );
    morphologyEx(closeFrame,closeFrame,cv::MORPH_CLOSE,struct_element_2);


    cv::Mat closeFramecp = cv::Mat(zoomFrame.size(), CV_8UC1, cv::Scalar(0));
    closeFrame.copyTo(closeFramecp);

    cv::Mat skel = cv::Mat::zeros(zoomFrame.size(), CV_8UC1);
    cv::Mat temp = cv::Mat(zoomFrame.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat eroded = cv::Mat(zoomFrame.size(), CV_8UC1, cv::Scalar(0));

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
        cv::erode(closeFramecp, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(closeFramecp, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(closeFramecp);

        done = (cv::countNonZero(closeFramecp) == 0);
    } while (!done);

    closeFrame.copyTo(zoomFrame);
    for (int i = 0; i < closeFrame.rows; i++){
        for (int j =0; j < closeFrame.cols; j++){
            if (skel.at<unsigned char>(i,j)  > 0) zoomFrame.at<unsigned char>(i,j) = 0 ;

        }
    }

    //skel.copyTo(closeFrame);
    update();
}
