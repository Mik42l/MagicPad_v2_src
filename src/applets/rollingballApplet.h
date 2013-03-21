#ifndef ROLLINGBALLAPPLET_H
#define ROLLINGBALLAPPLET_H

#include "applet.h"

#include <QPainter>
#include <QLCDNumber>
#include <QHBoxLayout>
#include <QTimer>
#include <QTime>
#include <QSound>

#include "filters/calibrationFilter.h"

#define ROLLINGBALL_NAME "rollingball"
#define ROLLINGBALL_TITLE "Rolling Ball"

// RollingBall Sound
#define ROLLINGBALL_SND_OOPS     "data/sound/oops.wav"
#define ROLLINGBALL_SND_GAMEOVER "data/sound/gameover.wav"
#define ROLLINGBALL_SND_COIN     "data/sound/chaching.wav"
#define ROLLINGBALL_SND_WOOP     "data/sound/woop.wav"

// Defines how much time a coins stays alive (ie visible)
// and how much time happend between two coins.
#define ROLLINGBALL_COIN_LIFETIME 7500.0   // 6 seconds
#define ROLLINGBALL_COIN_CYCLE    9500.0  // 10 seconds

/**
 *
 */
class RollingballApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    RollingballApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

    void reset();

    void paintEvent(QPaintEvent *);

    void start() {
        startGame();
    }

    void stop() {
        stopGame();
    }

    void respawn();

    void startGame();

    void stopGame();

    void addCoin();

    void getCoin();

    void killPlayer(bool outside = false);

private:

    double computeAngle();

private:

    CalibrationFilter *mCalibrationFilter;
    cv::Mat mFrame;

    // Physics
    double mAlpha; // plate angle
    double mSpeed; // ball x-speed;
    double mStep;  // ball x-position;
    bool mDead; // player is dead

    // GUI
    QLCDNumber *mLcd;
    QPixmap mImgBall;
    QPixmap mImgReflexion;
    QPixmap mImgShadow;
    QPixmap mImgCoin;
    int W; // board width;
    int ballRadius;

    // sensor
    int sensorleft_row;
    int sensorleft_col;
    int sensorright_row;
    int sensorright_col;
    bool sensornumber;

    // Game
    int mLives;
    int mScore;
    bool mGameOver;
    bool mCoinVisible;
    double mCoinPos;
    double mCoinCoeff;
    QTimer *mTmrCoinLife;
    QTimer *mTmrCoinCycle;
};

#endif // ROLLINGBALLAPPLET_H
