#include "rollingballApplet.h"

const QString RollingballApplet::TAG = QString("RollingballApplet");

/**
 *
 */
RollingballApplet::RollingballApplet(QWidget *parent) : Applet(parent)
{
    mName = ROLLINGBALL_NAME;
    mTitle = ROLLINGBALL_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    //
    // Build filter pipeline
    //
    mCalibrationFilter = new CalibrationFilter();
    reset();

    //
    // User interface
    //
    QPalette palette = this->palette();
    palette.setColor( backgroundRole(), Qt::gray );
    setPalette( palette );

    // images
    mImgBall.load( ":/image/rollingball/spirale.png" );
    mImgReflexion.load( ":/image/rollingball/reflexion.png" );
    mImgShadow.load( ":/image/rollingball/ballShadow.png" );
    mImgCoin.load( ":/image/rollingball/coin.png" );

    QHBoxLayout *layout = new QHBoxLayout( this );
    layout->setAlignment( Qt::AlignTop | Qt::AlignRight );

    mLcd = new QLCDNumber( this );
    mLcd->setNumDigits( 2 );
    mLcd->setFixedHeight( 32 );
    layout->addWidget( mLcd );

    mTmrCoinLife = new QTimer( this );
    mTmrCoinLife->setSingleShot( true );
    connect( mTmrCoinLife, SIGNAL( timeout() ), this, SLOT( killPlayer() ) );

    mTmrCoinCycle = new QTimer( this );
    mTmrCoinCycle->setInterval( ROLLINGBALL_COIN_CYCLE);
    connect( mTmrCoinCycle, SIGNAL( timeout() ), this, SLOT( addCoin() ) );

    qsrand( QTime::currentTime().msec() );

    // Gestures
    setGestures(SWAP_LEFT_RIGHT);
}

/**
 *
 */
void RollingballApplet::setFrame( cv::Mat& frame )
{
    // Filter pipeline
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();
    mFrame = mCalibrationFilter->getCalibratedFrame();

    // compute plate angle
    mAlpha = computeAngle();

    //
    // compute new ball position
    //
    if(!mDead)
    {
        mSpeed *= 0.98;                                // frottement
        mSpeed += -6.0 * sin(M_PI/180.0 * mAlpha);    // acceleration
        mSpeed = qBound(-20.0, mSpeed, 20.0);           // limit speed

        mStep += mSpeed;
        if(mStep > W)
        {
            mStep = W;
            killPlayer(true);
        }
        if(mStep < -W)
        {
            mStep = -W;
            killPlayer(true);
        }
    }

    update();
}

/**
 *
 */
void RollingballApplet::reset()
{
    mCalibrationFilter->reset();
}


/**
 * Add a new coin to the game
 */
void RollingballApplet::addCoin()
{
    //if(mGameOver) return;
    mCoinVisible = true;
    QSound::play(ROLLINGBALL_SND_WOOP);

    // find suitable coin position
    do
    {
        mCoinPos = (qrand() * 2.0 * W)/((double)RAND_MAX) - W;
    } while(std::abs(mStep - mCoinPos) < 3*ballRadius);


    // Lose a live if coin is not taken
    mTmrCoinLife->start(ROLLINGBALL_COIN_LIFETIME * mCoinCoeff);
    mCoinCoeff = std::max(0.3, mCoinCoeff-0.1);

    // start cycle
    mTmrCoinCycle->setInterval(ROLLINGBALL_COIN_CYCLE * mCoinCoeff);
    mTmrCoinCycle->stop();
    mTmrCoinCycle->start();
}

/**
 *  Compute mainPlate angle.
 * \param[out] angle Angle of the main plate in degree.
 */
double RollingballApplet::computeAngle()
{
    const double ANGLE_MAX = 30.0;

    double right = cv::mean( mFrame.colRange( cv::Range(6, 10) ) )[0];
    double left = cv::mean( mFrame.colRange( cv::Range(0, 5) ) )[0];

    double angle = (right-left)/10;

    return qBound(-ANGLE_MAX, angle, ANGLE_MAX);
}

/**
 * Add a new coin to the game
 */
void RollingballApplet::getCoin()
{
    // stop coin lifeTime timer
    mTmrCoinLife->stop();
    mCoinVisible = false;
    mSpeed /= 2;

    // add a point
    mScore++;
    mLcd->display( mScore );

    // play a sound
    QSound::play( ROLLINGBALL_SND_COIN );
}

/**
 * Kill player
 */
void RollingballApplet::killPlayer(bool outside)
{
    mTmrCoinLife->stop();
    mCoinVisible = false;
    mDead = true;
    mLives--;

    if(mLives > 0)
    {
        QSound::play(ROLLINGBALL_SND_OOPS);

        // Died from going out of the board
        if(outside)
        {
            mTmrCoinCycle->stop();
            QTimer::singleShot( 1500, this, SLOT( respawn() ) );
            QTimer::singleShot( 2000, this, SLOT( addCoin() ) );
        }
        else
        {
            mDead = false;
        }
    }
    else
    {
        QSound::play(ROLLINGBALL_SND_GAMEOVER);
        stopGame();
        QTimer::singleShot( 5000, this, SLOT( startGame() ) );
    }
}

/**
 * Reload player in the center
 */
void RollingballApplet::respawn()
{
    //if(mGameOver) return;
    mDead = false;
    mSpeed = 0;
    mStep = 0;
}

/**
 * Start the game
 */
void RollingballApplet::startGame()
{
    // dirty hack to prevent missing event
    if( !this->isVisible() )
    {
        stopGame();
        return;
    }

    // init game variables
    mGameOver = false;
    mScore = 0;
    mLives = 3;
    mLcd->display( mScore );
    mCoinCoeff = 1.0;
    mCoinVisible = false;
    this->respawn();
    QTimer::singleShot( 1000, this, SLOT( addCoin() ) );
}


/**
 * Stop the game
 */
void RollingballApplet::stopGame()
{
    mLives = 0; // Kill the player
    mGameOver = true;
    mTmrCoinLife->stop();
    mTmrCoinCycle->stop();
}


/**
 * Paint event method for window display.
 */
void RollingballApplet::paintEvent(QPaintEvent *)
{
    QPainter painter( this );
    painter.setRenderHint( QPainter::Antialiasing );

    //
    // Background
    //
    QPointF start(0, height());
    QPointF stop(0, 0);
    //QPointF stop = start - this->height()*QPointF(sin(M_PI/180.0 * (-m_alpha/2.0)),
    //                                              cos(M_PI/180.0 * (-m_alpha/2.0)));
    QLinearGradient g(start, stop);
    g.setColorAt( 0, Qt::black );
    g.setColorAt( 1, Qt::white );
    painter.setBrush( QBrush(g) );
    painter.drawRect( rect() );

    //
    // Draw Lives
    //
    painter.save();
    for(int i=0; i<mLives; ++i)
    {
        painter.drawPixmap(10, 10, 32, 32, mImgBall);
        painter.drawPixmap(10, 10, 32, 32, mImgShadow);
        painter.drawPixmap(10, 10, 32, 32, mImgReflexion);
        painter.translate(40, 0);
    }
    painter.restore();

    //
    // Display Game Over
    //
    if( mGameOver )
    {
        painter.setPen(Qt::red);
        painter.setBrush(Qt::red);
        painter.setFont(QFont("Times", 48, QFont::Bold));
        QRect r = this->rect();
        r.setHeight(this->height()/2);
        painter.drawText(r, Qt::AlignCenter, QString("GAME OVER"));
    }

    //
    // Draw plate depending on angle
    //
    W = 0.9 * this->width()/2.0;    // half width of the main plate
    const int H = 0.05 * this->height()/2.0;  // height of the main plate
    ballRadius = 2.0 * H;   // radius of the ball

    // Point p0: center of mainPlate
    const int x0 = this->width()/2;
    const int y0 = this->height()/2;

    //
    // Check for Ball / Coin collision
    //
    if( mCoinVisible )
    {
        if(std::abs(mStep - mCoinPos) < 1.5*ballRadius)
        {
            this->getCoin();
        }
    }

    //
    // Draw main plate
    //
    QPen pen;
    pen.setColor( Qt::black );
    pen.setCapStyle( Qt::RoundCap );
    pen.setWidth( 2 );
    painter.setPen( pen );
    painter.setBrush( Qt::green );
    painter.translate( x0, y0 );
    painter.rotate( -mAlpha );
    painter.drawRect( -W, 0, 2*W, 2*H );

    //
    // draw the coin
    //
    if(mCoinVisible)
    {
        painter.save();
        painter.translate( mCoinPos, - ballRadius );
        QRect r( -ballRadius/2, -ballRadius/2, ballRadius, ballRadius );
        painter.drawPixmap( r, mImgCoin );
        painter.restore();
    }

    //
    // draw the ball
    //
    ///painter.save();
    painter.translate( mStep, - ballRadius );
    QRect r( -ballRadius, -ballRadius, 2*ballRadius, 2*ballRadius );
    // draw ball
    painter.save();
    painter.rotate( (180.0 / M_PI) * ( mStep / ballRadius ) );
    painter.drawPixmap( r, mImgBall );
    painter.restore();
    // draw light reflexion and shadow
    painter.drawPixmap (r, mImgShadow );
    painter.drawPixmap( r, mImgReflexion );
    //painter.restore();

}
