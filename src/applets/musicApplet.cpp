#include "musicApplet.h"

const QString MusicApplet::TAG = QString("MusicApplet");

/**
 *
 */
MusicApplet::MusicApplet(QWidget *parent) : Applet(parent)
{
    mName = MUSIC_NAME;
    mTitle = MUSIC_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // build UI
    // Audio part
    QString audioFile( qApp->applicationDirPath() + MUSIC_SOUND_FILE );
    m_mediaObject = new Phonon::MediaObject(this);
    m_mediaObject->setCurrentSource(Phonon::MediaSource(audioFile));
    m_mediaObject->setTickInterval(1000);
    connect(m_mediaObject, SIGNAL(tick(qint64)), this, SLOT(tick(qint64)));
    connect(m_mediaObject, SIGNAL(aboutToFinish()), this, SLOT(loopMusic()));

    m_seekSlider = new Phonon::SeekSlider(this);
    m_seekSlider->setMediaObject(m_mediaObject);

    m_audioOutput = new Phonon::AudioOutput(Phonon::MusicCategory, this);
    m_path = Phonon::createPath(m_mediaObject, m_audioOutput);
    m_audioOutput->setMuted(false);

    // Build slider
    m_slider = new QSlider( Qt::Vertical );
    m_slider->setMinimum( 0 );
    m_slider->setMaximum( 100 );
    m_slider->setValue( 50 );
    m_slider->setTickPosition( QSlider::TicksBothSides );
    m_slider->setTickInterval( 10 );
    connect( m_slider, SIGNAL(valueChanged(int)), this, SLOT(changeVolume(int)) );

    // PLAY/PAUSE/STOP mode selection
    playButton= new QPushButton( this );
    playButton->setIcon( QIcon(":image/music/play.png") );
    connect(playButton, SIGNAL(clicked()), m_mediaObject, SLOT(play()));

    pauseButton= new QPushButton( this );
    pauseButton->setIcon( QIcon(":image/music/pause.png") );
    pauseButton->setEnabled( false );
    connect(pauseButton, SIGNAL(clicked()), m_mediaObject, SLOT(pause()));

    stopButton = new QPushButton( this );
    stopButton->setIcon( QIcon(":image/music/stop.png") );
    stopButton->setEnabled( false );
    connect( stopButton, SIGNAL(clicked()), m_mediaObject, SLOT(stop()) );
    connect( playButton, SIGNAL(clicked()), this, SLOT(setStoppingButton()) );
    connect( pauseButton, SIGNAL(clicked()), this, SLOT(setPlayingButton()) );
    connect( stopButton, SIGNAL(clicked()), this, SLOT(setPlayingButton()) );

    // Time display
    m_time = new QLabel("00:00");

    // CD Cover
    QLabel *cover = new QLabel( this );
    cover->setScaledContents( true );
    cover->setPixmap( QPixmap( MUSIC_COVER ) );

    // Main layout
    QGridLayout *mainLayout = new QGridLayout( this );
    mainLayout->addWidget( m_slider, 0, 0, 2, 1 );
    mainLayout->addWidget( cover, 0, 1, 1, 5 );
    mainLayout->addWidget( playButton, 1, 1 );
    mainLayout->addWidget( pauseButton, 1, 2 );
    mainLayout->addWidget( stopButton, 1, 3 );
    mainLayout->addWidget( m_time, 1, 4 );
    mainLayout->addWidget( m_seekSlider, 1, 5 );

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter( this );
    mMeanFilter = new MeanFilter( this );
    mObjectDetectionFilter = new ObjectDetectionFilter( this );

    // When an object leaves the MagicPad set the vumeter to the (n-10)th sample
    connect(mObjectDetectionFilter, SIGNAL(objectLeaves()), this, SLOT(resurect()));

    mLastValue = 0.0;

    // Gestures
    setGestures(UP_DOWN);
}

/**
 *
 */
void MusicApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    cv::Mat center = cv::Mat(mCalibrationFilter->getCalibratedFrame(), cv::Range(2,8), cv::Range(2,8) );

    mMeanFilter->setFrame( center );
    mMeanFilter->process();

    mObjectDetectionFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mObjectDetectionFilter->process();

    // If an object is detected over the sensor then the vumeter value
    // is updated continuously.
    if( mObjectDetectionFilter->getObjectDetected() )
    {
        double value = mMeanFilter->getMean() * 100.0 / 255.0;
        mLastValue += (value - mLastValue)*0.25;

        // Record sample
        mList.push_back( mLastValue );
        while(mList.size() > MUSIC_RESURECT_LENGTH) mList.pop_front();

        m_slider->setValue( mLastValue );
    }

    update();
}

/**
 *
 */
void MusicApplet::reset()
{
    mCalibrationFilter->reset();
    mMeanFilter->reset();
    mObjectDetectionFilter->reset();

    mList.clear();
}

/**
 *
 */
void MusicApplet::resurect()
{
    //QLOG_TRACE() << TAG << "resurect()";
    if( !mList.empty() )
    {
        m_slider->setValue( mList.front() );
    }
}


/**
 *
 */
void MusicApplet::changeVolume(int volume)
{
    m_audioOutput->setVolume(volume/100.0);
}

/**
 *
 */
void MusicApplet::closeEvent(QCloseEvent *event)
{
   this->changeVolume(0);
   event->accept();
}

/**
 *
 */
void MusicApplet::loopMusic()
{
    m_mediaObject->seek(0);
}

/**
 *
 */
void MusicApplet::tick(qint64 time)
{
     QTime displayTime(0, (time / 60000) % 60, (time / 1000) % 60);
     m_time->setText(displayTime.toString("mm:ss"));
}
