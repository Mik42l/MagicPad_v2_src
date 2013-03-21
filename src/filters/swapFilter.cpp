#include "swapFilter.h"


const QString SwapFilter::TAG = QString("SwapFilter");

/**
 *
 */
SwapFilter::SwapFilter( QObject *parent ) :
    Filter( parent )
{
    setupStateMachine();

    reset();
}

/**
 *
 */
void SwapFilter::process()
{
    // reset LeftToRightSwap / RightToLeftSwap
    LeftToRightSwap = false;
    RightToLeftSwap = false;

    unsigned char L, R;     // Left and Rigth area value

     L = cv::mean( Frame( cv::Range(5,10), cv::Range(0, 3)) )[0];
     R = cv::mean( Frame( cv::Range(5,10), cv::Range(7,10)))[0];

    if( L > mThresHigh && R > mThresHigh )
    {
        emit bothHigh();
    }
    else if( L > mThresHigh )
    {
        emit highLeft();
    }
    else if( R > mThresHigh )
    {
        emit highRight();
    }

    if( L < mThresLow )
    {
        emit lowLeft();
    }
    if( R < mThresLow )
    {
        emit lowRight();
    }
}

/**
 *
 */
void SwapFilter::reset()
{
    //threshold values
    mThresHigh = SWAPFILTER_THRESHOLD_HIGH;
    mThresLow = SWAPFILTER_THRESHOLD_LOW;
}

/**
 *
 */
void SwapFilter::setupStateMachine()
{
    // Timer: if swap is longer that 1000ms then it is discarded
    QTimer *timer = new QTimer(this);
    timer->setInterval(750);
    timer->setSingleShot(true);

    // Movement state machine
    QState *E0 = new QState();
    QState *Et = new QState();
    QState *Er1 = new QState();
    QState *Er2 = new QState();
    QState *Er3 = new QState();
    QState *El1 = new QState();
    QState *El2 = new QState();
    QState *El3 = new QState();

    mFSM.addState(E0);
    mFSM.addState(Et);
    mFSM.addState(Er1);
    mFSM.addState(Er2);
    mFSM.addState(Er3);
    mFSM.addState(El1);
    mFSM.addState(El2);
    mFSM.addState(El3);
    mFSM.setInitialState(E0);

    /* STATE MACHINE */
    // E0
    connect(E0, SIGNAL(entered()), timer, SLOT(stop()));

    // Et
    Et->addTransition(this, SIGNAL(bothHigh()), E0);

    // --- Left-to-right swap ---
    // E0 -> Er1
    E0->addTransition(this, SIGNAL(lowLeft()), Er1);

    // Er1
    connect(Er1, SIGNAL(entered()), timer, SLOT(start()));

    // Er1 -> E0
    Er1->addTransition(timer, SIGNAL(timeout()), Et);

    // Er1 -> Er2
    Er1->addTransition(this, SIGNAL(lowRight()), Er2);

    // Er2 -> E0
    Er2->addTransition(timer, SIGNAL(timeout()), Et);

    // Er2 -> Er3
    Er2->addTransition(this, SIGNAL(bothHigh()), Er3);

    // Er3
    connect(Er3, SIGNAL(entered()), timer, SLOT(stop()));
    connect(Er3, SIGNAL(entered()), this, SLOT(l2r()));

    // Er3 -> E0
    Er3->addTransition(E0);

    // --- Right-to-left swap ---
    // E0 -> El1
    E0->addTransition(this, SIGNAL(lowRight()), El1);

    // El1
    connect(El1, SIGNAL(entered()), timer, SLOT(start()));

    // El1 -> E0
    El1->addTransition(timer, SIGNAL(timeout()), Et);

    // El1 -> El2
    El1->addTransition(this, SIGNAL(lowLeft()), El2);

    // El2 -> E0
    El2->addTransition(timer, SIGNAL(timeout()), Et);

    // El2 -> El3
    El2->addTransition(this, SIGNAL(bothHigh()), El3);

    // El3
    connect(El3, SIGNAL(entered()), timer, SLOT(stop()));
    connect(El3, SIGNAL(entered()), this, SLOT(r2l()));

    // El3 -> E0
    El3->addTransition(E0);

    mFSM.start();/* STATE MACHINE */
    /*// E0
    connect(E0, SIGNAL(entered()), timer, SLOT(stop()));

    // Et
    Et->addTransition(this, SIGNAL(bothHigh()), E0);

    // --- Left-to-right swap ---
    // E0 -> Er1
    E0->addTransition(this, SIGNAL(lowLeft()), Er1);

    // Er1
    connect(Er1, SIGNAL(entered()), timer, SLOT(start()));

    // Er1 -> E0
    Er1->addTransition(timer, SIGNAL(timeout()), Et);

    // Er1 -> Er2
    Er1->addTransition(this, SIGNAL(lowRight()), Er2);

    // Er2 -> E0
    Er2->addTransition(timer, SIGNAL(timeout()), Et);

    // Er2 -> Er3
    Er2->addTransition(this, SIGNAL(bothHigh()), Er3);

    // Er3
    connect(Er3, SIGNAL(entered()), timer, SLOT(stop()));
    connect(Er3, SIGNAL(entered()), this, SLOT(l2r()));

    // Er3 -> E0
    Er3->addTransition(E0);

    // --- Right-to-left swap ---
    // E0 -> El1
    E0->addTransition(this, SIGNAL(lowRight()), El1);

    // El1
    connect(El1, SIGNAL(entered()), timer, SLOT(start()));

    // El1 -> E0
    El1->addTransition(timer, SIGNAL(timeout()), Et);

    // El1 -> El2
    El1->addTransition(this, SIGNAL(lowLeft()), El2);

    // El2 -> E0
    El2->addTransition(timer, SIGNAL(timeout()), Et);

    // El2 -> El3
    El2->addTransition(this, SIGNAL(bothHigh()), El3);

    // El3
    connect(El3, SIGNAL(entered()), timer, SLOT(stop()));
    connect(El3, SIGNAL(entered()), this, SLOT(r2l()));

    // El3 -> E0
    El3->addTransition(E0);

    mFSM.start();*/
}
