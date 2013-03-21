#include "slideshowApplet.h"

const QString SlideshowApplet::TAG = "SlideshowApplet";

/**
 *
 */
SlideshowApplet::SlideshowApplet(QWidget *parent) : Applet(parent)
{
    mName = SLIDESHOW_NAME;
    mTitle = SLIDESHOW_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Filter pipeline
    mCalibrationFilter = new CalibrationFilter( this );
    mSwapFilter = new SwapFilter( this );

    connect( mSwapFilter, SIGNAL(leftToRight()), this, SLOT(leftToRight()) );
    connect( mSwapFilter, SIGNAL(rightToLeft()), this, SLOT(rightToLeft()) );

    // GUI
    QPushButton *btn = new QPushButton( "Open document", this );
    resize( parent->size() );
    btn->move( rect().center() );
    btn->setMinimumSize( 230, 80 );

    QFont f = btn->font();
    f.setPointSize( 14 );
    btn->setFont( f );
    connect( btn, SIGNAL(clicked()), this, SLOT(openFile()) );

    mArrowWidget = new ArrowHintWidget( this );

    // Gestures
    setGestures(SWAP_LEFT_RIGHT);
}

/**
 *
 */
void SlideshowApplet::openFile()
{
    // User select a file
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open document"),
        QDir::currentPath(),
        tr("Microsoft PowerPoint, Adobe PDF(*.ppt *.pdf)") );

    if( fileName.isNull() )
    {
        return;
    }

    // Run external viewer
    QUrl url = QUrl::fromLocalFile( fileName );
    if( !QDesktopServices::openUrl( url ) )
    {
        QMessageBox msgBox;
        msgBox.setText(QString("Unable to open %1").arg(url.toString()));
        msgBox.exec();
        return;
    }
}

/**
 * Send a command (keyboard stroke) the the current active window
 * keyaction:  0 -> Press & Release
 *             1 -> Press
 *            -1 -> Release
 */
void SlideshowApplet::sendCommand( int character, int keyaction )
{
    INPUT *key;
    key = new INPUT;
    key->type = INPUT_KEYBOARD;
    key->ki.wVk = character;
    key->ki.dwFlags = 0; // Key pressed
    key->ki.wScan = MapVirtualKey(character, 0);

    if(keyaction==0 || keyaction==1)
    {
        SendInput(1, key, sizeof(INPUT));
    }

    // Key Up event
    if(keyaction==0 || keyaction==-1)
    {
        key->ki.dwFlags = KEYEVENTF_KEYUP;
        Sleep(1);
        SendInput(1, key, sizeof(INPUT));
    }
}

/**
 *
 */
void SlideshowApplet::setFrame( cv::Mat& frame )
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mSwapFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mSwapFilter->process();
}


/**
 * Process left->right movement
 */
void SlideshowApplet::leftToRight()
{
    // Todo show --> overlay
    sendCommand(VK_LEFT, 0);

    mArrowWidget->popup( ArrowHintWidget::ARROW_LEFT );
}

/**
 * Process right->left movement
 */
void SlideshowApplet::rightToLeft()
{
    // Todo: show <-- overlay
    sendCommand(VK_RIGHT, 0);

    mArrowWidget->popup( ArrowHintWidget::ARROW_RIGHT );
}
