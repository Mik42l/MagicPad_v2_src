#include "alphabetApplet.h"

const QString AlphabetApplet::TAG = QString("AlphabetApplet");

/**
 *
 */
AlphabetApplet::AlphabetApplet(QWidget *parent) : Applet(parent)
{
    mName = ALPHABET_NAME;
    mTitle = ALPHABET_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();

    // Build filter pipeline
    mCalibrationFilter = new CalibrationFilter();
    mAlphabetFilter = new AlphabetFilter();

    //
    // Build GUI
    //

    // load letters
    bgPix.load(":/image/alphabet/background.jpg");
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/A.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/G.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/H.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/I.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/K.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/L.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/O.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/R.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/S.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/T.png"));
    pixmaps << new Pixmap(QPixmap(":/image/alphabet/W.png"));

    scene = new QGraphicsScene( -350, -350, 700, 700, this );

    for (int i = 0; i <  pixmaps.size(); ++i) {
        pixmaps[i]->setOffset(-pixmaps[i]->pixmap().width()/2, -pixmaps[i]->pixmap().height()/2);
        pixmaps[i]->setZValue(i);
        items << pixmaps[i];
        scene->addItem(pixmaps[i]);
    }

    rootState = new QState;
    ellipseState = new QState(rootState);
    centeredState = new QState(rootState);
    for(int i=0; i<pixmaps.size(); i++)
    {
        statesLetters << new QState(rootState);
    }

    // Random vector permutation:
    // Allow the letters to be in a different configuration
    // each time.
    std::vector<int> randperm;
    for(int i=0; i<pixmaps.size(); i++) {
        randperm.push_back(i);
    }

    srand( QTime::currentTime().second() );
    std::random_shuffle( randperm.begin(), randperm.end() );

    // State positions
    for (int i = 0; i < items.count(); ++i) {
        Pixmap *item = items.at(i);
        // Ellipse (idle state)
        QPoint p(cos((randperm[i] / (double)pixmaps.size()) * 6.28) * 300,
                 sin((randperm[i] / (double)pixmaps.size()) * 6.28) * 300);
        ellipseState->assignProperty(item, "pos", p);
        ellipseState->assignProperty(item, "scale", 0.4);

        // met a jours les Etats correspondant aux differentes lettres
        for(int i=0; i<pixmaps.size(); i++)
        {
            statesLetters.at(i)->assignProperty(item, "pos", p);
            statesLetters.at(i)->assignProperty(item, "scale", 0.4);
        }

        // Centered
        centeredState->assignProperty(item, "pos", QPointF(0, 0));
        centeredState->assignProperty(item, "scale", 0.4);
    }

    // mettre une des lettre au centre
    for(int i=0; i<pixmaps.size(); i++)
    {
        statesLetters.at(i)->assignProperty(items.at(i), "pos", QPointF(0, 0));
        statesLetters.at(i)->assignProperty(items.at(i), "scale", 1.00);
    }

    // Ui
    view = new AlphabetView( scene );
    //view->setViewportUpdateMode( QGraphicsView::BoundingRectViewportUpdate );
    view->setBackgroundBrush( bgPix );
    //view->setCacheMode( QGraphicsView::CacheBackground );
    //view->setRenderHints( QPainter::Antialiasing | QPainter::SmoothPixmapTransform );
    view->show();
    //view->setFixedSize(view->size());

    QVBoxLayout *layout = new QVBoxLayout( this );
    layout->addWidget(view);
    //QLOG_DEBUG() << TAG << "view size()" << view->size();

    states.addState(rootState);
    states.setInitialState(rootState);
    rootState->setInitialState(centeredState);

    group = new QParallelAnimationGroup;
    for (int i = 0; i < items.count(); ++i) {
        QPropertyAnimation *animPos = new QPropertyAnimation(items[i], "pos");
        animPos->setDuration(1000 + randperm[i]*20);
        animPos->setEasingCurve(QEasingCurve::InOutBack);
        group->addAnimation(animPos);

        QPropertyAnimation *animSize = new QPropertyAnimation(items[i], "scale");
        animSize->setDuration(1000 + randperm[i]*20);
        animSize->setEasingCurve(QEasingCurve::InOutBack);
        group->addAnimation(animSize);
    }

    int idx = 0;
    trans = rootState->addTransition(this, SIGNAL(trigger_A()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_G()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_H()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_I()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_K()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_L()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_O()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_R()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_S()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_T()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_W()), statesLetters[idx++]);
    trans->addAnimation(group);

    trans = rootState->addTransition(this, SIGNAL(trigger_unknow()), ellipseState);
    trans->addAnimation(group);

    mStartTimer = new QTimer();
    mStartTimer->setSingleShot( true );
    trans = rootState->addTransition( mStartTimer, SIGNAL(timeout()), ellipseState );
    trans->addAnimation( group );
    mStartTimer->start( 200 );

    states.start();

    // Gestures
    setGestures(ALPHABET);
}

/**
 *
 */
void AlphabetApplet::setFrame(cv::Mat &frame)
{
    mCalibrationFilter->setFrame( frame );
    mCalibrationFilter->process();

    mAlphabetFilter->setFrame( mCalibrationFilter->getCalibratedFrame() );
    mAlphabetFilter->process();

    char c = mAlphabetFilter->getLetter();

    if( c != mLastChar )
    {
        //QLOG_DEBUG() << TAG << "Letter=" << c;
        switch( c )
        {
            case 'A': emit trigger_A(); break;
            case 'G': emit trigger_G(); break;
            case 'H': emit trigger_H(); break;
            case 'I': emit trigger_I(); break;
            case 'K': emit trigger_K(); break;
            case 'L': emit trigger_L(); break;
            case 'O': emit trigger_O(); break;
            case 'R': emit trigger_R(); break;
            case 'S': emit trigger_S(); break;
            case 'T': emit trigger_T(); break;
            case 'W': emit trigger_W(); break;
            case '?': default:
            emit trigger_unknow();
        }
    }

    mLastChar = c;

    update();
}
