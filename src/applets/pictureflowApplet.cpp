#include "pictureflowApplet.h"

const QString PictureFlowApplet::TAG = QString("PictureFlowApplet");

/**
 *
 */
PictureFlowApplet::PictureFlowApplet(QWidget *parent) : Applet(parent)
{
    mName = PICTUREFLOW_NAME;
    mTitle = PICTUREFLOW_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter( this );

    mSwapFilter = new SwapFilter( this );

    // Build UI
    mPictureFlow = new PictureFlow( this );
    mPictureFlow->setSlideSize( QSize(400, 400) );
    //mPictureFlow->resize( parent->size() );

    QStringList files = findFiles( qApp->applicationDirPath() +  PICTUREFLOW_IMAGES_PATH );

    QImage img;
    for(int i=0; i <(int)files.count(); i++)
    {
        if( img.load(files[i]) )
        {
            mPictureFlow->addSlide( img );
        }
    }
    mPictureFlow->setCenterIndex( mPictureFlow->slideCount() / 2 );
    mPictureFlow->setBackgroundColor( Qt::black );
    //mPictureFlow->setBackgroundColor( Qt::transparent );
    mPictureFlow->setReflectionEffect( PictureFlow::PlainReflection );

    // Gestures
    setGestures(SWAP_LEFT_RIGHT);
}

/**
 *
 */
void PictureFlowApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mSwapFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mSwapFilter->process();

    if( mSwapFilter->getLeftToRightSwap() )
    {
        mPictureFlow->showPrevious();
    }
    else if( mSwapFilter->getRightToLeftSwap() )
    {
        mPictureFlow->showNext();
    }

    //update();
}

/**
 *
 */
QStringList PictureFlowApplet::findFiles( const QString& path )
{
    QStringList files;

    QDir dir = QDir::current();
    if (!path.isEmpty())
        dir = QDir(path);

    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    QFileInfoList list = dir.entryInfoList();
    for (int i = 0; i < list.size(); ++i)
        files += dir.absoluteFilePath(list.at(i).fileName());

    return files;
}
