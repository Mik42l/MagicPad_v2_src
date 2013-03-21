#include "mainwindow.h"

const QString MainWindow::TAG = QString("MainWindow");

/**
 *
 */
MainWindow::MainWindow(QWidget *parent)
    : QWidget(parent)
{
    // === Preferences === //
    loadSettings();

    // === Connection === //
    MagicPad *magicPad = new MagicPad();
    mProducer = new FrameProducer();

    int force_com_port = mSettings->value("Communication/force_com_port", 0).toInt();
    if(force_com_port)
    {
        MagicPadDevice device(force_com_port, "MagicPad on COM" + QString::number(force_com_port));
        mProducer->setDevice(device);
        QLOG_INFO() << TAG << "Forcing MagicPad on COM" << force_com_port;
        mProducer->start();
    }
    else
    {
        QList<MagicPadDevice> L = magicPad->listDevices();
        QLOG_INFO() << L.size() << "MagicPad device(s) found";
        if(L.size() > 0)
        {
            mProducer->setDevice(L.first());
            mProducer->start();
        }
        else if(!mCanRunWithoutMagicPad)
        {
            QMessageBox msgBox;
            msgBox.setText("No MagicPad devices found.");
            msgBox.setInformativeText("Make sure that the MagicPad is turned on and the USB cable is plugged in. When using bluetooth, set the COM port number in config.ini");
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.exec();
            exit(0);
        }
        else {
            QMessageBox::warning(this, "No MagicPad devices found", "No MagicPad devices were found.");
        }
    }

    // === Main window === //
    setupUI();

    QPalette p;
    p = palette();
    p.setBrush( QPalette::Window, QBrush( QPixmap( ":image/metal.jpg" ) ) );
    setPalette( p );    
}

/**
 *
 */
void MainWindow::setupUI()
{
    // Maximize
    setWindowState( Qt::WindowFullScreen );

    // Logger
    mLogger = new LoggerDialog( this );

    mScreen = QApplication::desktop()->screenGeometry();

    QGraphicsScene *scene = new QGraphicsScene( this );
    scene->setSceneRect( mScreen );
    View *view = new View( scene );

    QVBoxLayout *layout = new QVBoxLayout( this );
    layout->setMargin( 0 );
    layout->addWidget( view );

    // === Version text === //
    // The version text appears at the bottom right of the screen.
    // It shows the program name and version and a link to www.isorg.fr
    QGraphicsTextItem *versionText = new QGraphicsTextItem;
    QFont ff = versionText->font();
    ff.setPointSize( 10 );
    versionText->setFont( ff );
    versionText->setDefaultTextColor( Qt::white );
    versionText->setHtml(QString(
        "MagicPad v%1 © ISORG 2012\n<a href=\"http://www.isorg.fr\" style=\"color: rgb(255,255,255)\">www.isorg.fr</a>"
        ).arg(MAGICPAD_VERSION_STR));
    //versionText->setOpenExternalLinks( true );
    //versionText->setTextInteractionFlags( Qt::TextBrowserInteraction );
    versionText->setGraphicsEffect( new AppletShadowEffect() );
    versionText->setTextWidth( 200 );
    versionText->setPos( mScreen.width() - 200, mScreen.height() - 40 );
    scene->addItem( versionText );

    // === Left column text === //
    // When an applet is launched a descriptive text drops down on the left
    // side of the screen. All applets share the same container for their
    // text: only the text is changed, the style remains.
    mText = new QGraphicsTextItem;
    QFont f = mText->font();
    f.setPointSize( 14 );
    mText->setFont( f );
    mText->setDefaultTextColor( Qt::white );
    mText->setHtml("This is the default text, it is replaced at runtime by the applet descriptive text");
    mText->setPos( 0, 0 );
    mText->setGraphicsEffect( new AppletShadowEffect() );  

    // Width of the description text and gesture icons
    mDescriptionTextWidth = qBound(300.0, mScreen.width() - mScreen.height() - 50, 400.0);
    mText->setTextWidth( mDescriptionTextWidth );

    if( !mShowText ) mText->hide();
    //scene->addItem( mText );

    // Create accepted gestures icon
    int gesSpacing = 22;   // spacing between pixmapItems
    int gesWidth = QPixmap(":image/icon_acc_ges_TWIST.png").width();
    int gesIconWidth = (mDescriptionTextWidth - 3*gesSpacing) / 3;
    double gesScale = (double)gesIconWidth / gesWidth;
    mAcceptedGestures.resize(3);
    for(int i=0; i<3; i++) {
        mAcceptedGestures[i] = new QGraphicsPixmapItem;
        mAcceptedGestures[i]->setPos( gesSpacing + i*gesIconWidth + (i+0.5)*gesSpacing, mScreen.height()-300 );
        mAcceptedGestures[i]->setScale( gesScale );
    }

    // Create group to hold the description text and the gestures icons
    mTextAnsGesturesGroup = new QGraphicsWidget;
    scene->addItem(mTextAnsGesturesGroup);
    mText->setParentItem(mTextAnsGesturesGroup);
    foreach(QGraphicsPixmapItem* p, mAcceptedGestures) {
        p->setParentItem(mTextAnsGesturesGroup);
    }

    // === Action Bar pannel === //
    // The Action Bar is the left vertical pannel that holds the back,
    // connect, quit and message buttons. The buttons are objects of
    // ActionButton type.
    QGraphicsRectItem *actionBar = new QGraphicsRectItem;
    actionBar->setRect( 0, 0, 50, mScreen.height() );
    actionBar->setPen( QPen( Qt::black ) );
    actionBar->setBrush( Qt::black );
    actionBar->setZValue( 100 );
    scene->addItem( actionBar );

    // Back button
    ActionButton *backButton = new ActionButton(QPixmap(":image/reverse.png").scaled(40, 40, Qt::IgnoreAspectRatio, Qt::SmoothTransformation), actionBar);
    backButton->setPos(5, 25);
    connect(backButton, SIGNAL(pressed()), this, SLOT(closeCurrentAppletLater()));
    connect(this, SIGNAL(emulateBackButton()), this, SLOT(closeCurrentAppletLater()));

    // Information button
    ActionButton *informationButton = new ActionButton(QPixmap(":image/information.png").scaled(40, 40, Qt::IgnoreAspectRatio, Qt::SmoothTransformation), actionBar);
    informationButton->setPos(5, mScreen.height()-230);
    informationButton->setState(mShowText);
    informationButton->setCheckable( true );
    connect(informationButton, SIGNAL(pressed()), this, SLOT(changeText()));

    // Logger button
    ActionButton *messageButton = new ActionButton(QPixmap(":image/message.png").scaled(40, 40, Qt::IgnoreAspectRatio, Qt::SmoothTransformation), actionBar);
    messageButton->setPos( 5, mScreen.height()-140 );
    if( !mShowLogger ) messageButton->hide();
    connect( messageButton, SIGNAL(pressed()), mLogger, SLOT(exec()) );

    // Quit button
    ActionButton *quitButton = new ActionButton(QPixmap(":image/power.png").scaled(40, 40, Qt::IgnoreAspectRatio, Qt::SmoothTransformation), actionBar);
    quitButton->setPos(5, mScreen.height()-50 );
    if( !mShowQuit ) quitButton->hide();
    connect( quitButton, SIGNAL(pressed()), this, SLOT(close()) );

    // === Applets === //
    // Applets are the independent demonstrations embedded in the MagicPad
    // software. Each applet as a name an an icon. When the applets are
    // loaded, their icons are displayed in a grid-like layout.
    mCurrentApplet = NULL;
    loadApplets( scene );

    // === Animation === //
    // All animations (=item movement, change of style, ...) are handled by a
    // QStateMachine that calls animation

    // State machine
    rootState = new QState;
    QState *homeState = new QState(rootState);
    QState *appState = new QState(rootState);
    QState *appStateWithText = new QState(appState);
    QState *appStateWithoutText = new QState(appState);


    QStateMachine *states = new QStateMachine;
    states->addState(rootState);
    states->setInitialState(rootState);
    rootState->setInitialState(homeState);

    // Home state
    homeState->assignProperty(mTextAnsGesturesGroup, "pos", QPointF(50, -mScreen.height()));

    QPointF origin(50 + mScreen.center().x() - (MAINWINDOW_APPLETGRID_NCOL*180.0)/2.0,
                   mScreen.center().y() - 180.0*0.5*(mApplets.size()/MAINWINDOW_APPLETGRID_NCOL));
    homeState->assignProperty(mAppletButtonGrid, "pos", origin);
    homeState->assignProperty(mAppletRect, "pos", QPointF(mScreen.width(), (mScreen.height()-mAppletRect->height())/2 ));
    homeState->assignProperty(backButton, "visible", false);
    homeState->assignProperty(informationButton, "visible", false);

    // App state
    appState->assignProperty(mAppletButtonGrid, "pos", origin - QPointF(mScreen.width(), 10));
    appState->assignProperty(mTextAnsGesturesGroup, "pos", QPointF(50, 150));
    if( mShowBack ) appState->assignProperty(backButton, "visible", true );
    if (mShowInformationButton) appState->assignProperty(informationButton, "visible", true);

    // App state with text
    appStateWithText->assignProperty( mAppletRect, "pos", QPointF( mDescriptionTextWidth + 75, (mScreen.height()-mAppletRect->height())/2 ) );
    appStateWithText->assignProperty( mTextAnsGesturesGroup, "visible", true);

    // App state without text
    appStateWithoutText->assignProperty( mAppletRect, "pos", QPointF( 100 + (mScreen.width() - mAppletRect->width())/2, (mScreen.height()-mAppletRect->height())/2) );
    appStateWithoutText->assignProperty( mTextAnsGesturesGroup, "visible", false);

    if (mShowText) appState->setInitialState(appStateWithText);
    else appState->setInitialState(appStateWithoutText);

    // Animations
    // HOME -> APP : app icons out, then text in
    QParallelAnimationGroup *slideAppletAnimation = new QParallelAnimationGroup;
    QParallelAnimationGroup *gotoAppStateAnimation = new QParallelAnimationGroup;



    QPropertyAnimation *animTextIn = new QPropertyAnimation(mTextAnsGesturesGroup, "pos");
    animTextIn->setDuration(750);
    animTextIn->setEasingCurve(QEasingCurve::OutBounce);

    QPropertyAnimation *animGridOut = new QPropertyAnimation(mAppletButtonGrid, "pos");
    animGridOut->setDuration(750);
    animGridOut->setEasingCurve(QEasingCurve::OutQuad);
    slideAppletAnimation->addAnimation(animGridOut);

    QPropertyAnimation *animAppIn = new QPropertyAnimation(mAppletRect, "pos");
    animAppIn->setDuration(750);
    animAppIn->setEasingCurve(QEasingCurve::OutQuad);
    slideAppletAnimation->addAnimation(animAppIn);

    QPauseAnimation *animTextInPause = new QPauseAnimation(350);

    QSequentialAnimationGroup *animTextInGroup = new QSequentialAnimationGroup;
    animTextInGroup->addAnimation(animTextInPause);
    animTextInGroup->addAnimation(animTextIn);

    gotoAppStateAnimation->addAnimation(animTextInGroup);
    gotoAppStateAnimation->addAnimation(slideAppletAnimation);

    gotoAppStateAnimation->addAnimation(animTextInGroup);

    // APP -> HOME : app icons in, then text out
    QParallelAnimationGroup *slideHomeAnimation = new QParallelAnimationGroup;
    QParallelAnimationGroup *gotoHomeStateAnimation = new QParallelAnimationGroup;

    QPropertyAnimation *animTextOut = new QPropertyAnimation(mTextAnsGesturesGroup, "pos");
    animTextOut->setDuration(750);
    animTextOut->setEasingCurve(QEasingCurve::OutQuad);
    gotoHomeStateAnimation->addAnimation(animTextOut);

    QPauseAnimation *animGridInPause = new QPauseAnimation(200);

    QPropertyAnimation *animGridIn = new QPropertyAnimation(mAppletButtonGrid, "pos");
    animGridIn->setDuration(750);
    animGridIn->setEasingCurve(QEasingCurve::OutQuad);

    QSequentialAnimationGroup *animGridInGroup = new QSequentialAnimationGroup;
    animGridInGroup->addAnimation(animGridInPause);
    animGridInGroup->addAnimation(animGridIn);

    slideHomeAnimation->addAnimation(animGridInGroup);

    QPropertyAnimation *animAppOut = new QPropertyAnimation(mAppletRect, "pos");
    animAppOut->setDuration(750);
    animAppOut->setEasingCurve(QEasingCurve::OutQuad);
    slideHomeAnimation->addAnimation(animAppOut);

    gotoHomeStateAnimation->addAnimation(slideHomeAnimation);


    // APP without Text -> APP with Text : text appears, appletRect shifts on the right
    QParallelAnimationGroup *animTextAppear = new QParallelAnimationGroup;

    QPropertyAnimation *textAppear = new QPropertyAnimation(mText, "opacity");
    textAppear->setDuration(750);
    textAppear->setStartValue(0.0);
    textAppear->setEndValue(1.0);

    QPropertyAnimation *animAppletToRight = new QPropertyAnimation(mAppletRect, "pos");
    animAppletToRight->setDuration(500);
    animAppletToRight->setEasingCurve(QEasingCurve::OutQuad);

    animTextAppear->addAnimation(textAppear);
    animTextAppear->addAnimation(animAppletToRight);


    // APP with Text -> APP without Text : text disappears, appletRect shifts on the left
    QParallelAnimationGroup *animTextDisappear = new QParallelAnimationGroup;

    QPropertyAnimation *animAppletToLeft = new QPropertyAnimation(mAppletRect, "pos");
    animAppletToLeft->setDuration(500);
    animAppletToLeft->setEasingCurve(QEasingCurve::OutQuad);

    animTextDisappear->addAnimation(animAppletToLeft);


    // Transitions
    QAbstractTransition *trans = rootState->addTransition(backButton, SIGNAL(pressed()), homeState);
    trans->addAnimation(gotoHomeStateAnimation);

    trans = rootState->addTransition(this, SIGNAL(emulateBackButton()), homeState);
    trans->addAnimation(gotoHomeStateAnimation);

    trans = rootState->addTransition(this, SIGNAL(goApplet()), appState);
    trans->addAnimation(gotoAppStateAnimation);

    trans = appState->addTransition(this, SIGNAL(goAppletWithText()), appStateWithText);
    trans->addAnimation(gotoAppStateAnimation);

    trans = appState->addTransition(this, SIGNAL(goAppletWithoutText()), appStateWithoutText);
    trans->addAnimation(gotoAppStateAnimation);

    trans = appState->addTransition(informationButton, SIGNAL(pressedOFF()), appStateWithoutText);
    trans->addAnimation(animTextDisappear);

    trans = appState->addTransition(informationButton, SIGNAL(pressedON()), appStateWithText);
    trans->addAnimation(animTextAppear);

    // Start state machine
    states->start();
}

/**
 * Instantiate all applets
 */
void MainWindow::loadApplets(QGraphicsScene *scene)
{
    //
    // Applet icon container:
    // This widget hold the applets icon.
    //
    mAppletButtonGrid = new QGraphicsWidget();
    scene->addItem( mAppletButtonGrid );

    //
    // Applet widget container:
    // This widget holds the applets display area
    //
    mAppletRect = new QWidget( this );

    int sz;
    if( mShowText ) {
        sz = mScreen.width() - 510;
    } else {
        sz = mScreen.width() - 200;
        sz = qMin( 1.3*(mScreen.height()-50), (double)sz );
    }
    mAppletRect->resize( sz, mScreen.height()-50 );

    //
    // Register applets to the applet manager.
    // Icon, names, etc... will be automatically handled from there!
    // Order of registration is important for applet position in the grid
    // Grid is filled from top-left, row-by-row
    //
    registerApplet( new AlphabetApplet( mAppletRect ) );
    registerApplet( new BacklightApplet( mAppletRect ) );
    registerApplet( new BargraphApplet( mAppletRect ) );
    registerApplet( new BulbApplet( mAppletRect ) );
    registerApplet( new DjApplet( mAppletRect ) );
    registerApplet( new GestureApplet( mAppletRect ) );
    registerApplet( new ImagedisplayApplet( mAppletRect ) );
    registerApplet( new MapsApplet( mAppletRect ) );
    registerApplet( new MusicApplet( mAppletRect ) );
    registerApplet( new OpdApplet( mAppletRect ) );
    registerApplet( new PictureFlowApplet( mAppletRect ) );
    //registerApplet( new PongApplet( mAppletRect ) );
    registerApplet( new PurpleApplet( mAppletRect ) );
    registerApplet( new RollingballApplet( mAppletRect ) );
    registerApplet( new SlideshowApplet( mAppletRect ) );
    registerApplet( new SurfaceApplet( mAppletRect ) );
    registerApplet( new SwitchApplet( mAppletRect ) );
    registerApplet( new TwistApplet( mAppletRect ) );
    registerApplet( new VumeterApplet( mAppletRect ) );
    registerApplet( new NewprojectApplet( mAppletRect ) );

    //
    // Load plugin applets
    //
    QDir pluginsDir( qApp->applicationDirPath() );
    pluginsDir.cd( "plugins" );

    int n = 0;
    foreach (QString fileName, pluginsDir.entryList(QDir::Files) )
    {
        QPluginLoader pluginLoader(pluginsDir.absoluteFilePath(fileName));
        QObject *plugin = pluginLoader.instance();
        if( plugin )
        {
            AppletInterface *papplet = qobject_cast<AppletInterface *>(plugin);
            if( papplet )
            {
                QLOG_INFO() << TAG << "Plugin" << papplet->title() << "loaded successfully";
                papplet->setParent( mAppletRect );
                registerApplet( papplet );
                n++;
            }
        }
    }

    //
    // Connect frame producer
    //
    connect(mProducer, SIGNAL(newFrame(cv::Mat &)), this, SLOT(dispatchFrame(cv::Mat&)));

    mReadyToLaunchApplet = true;
}

/**
 * Add an applet to the home menu, connect the sigals/slots
 */
void MainWindow::registerApplet(AppletInterface *applet)
{
    // Resize applet
    applet->resize( mAppletRect->size() );
    applet->hide();

    // Add applet to the list
    mApplets.push_back( applet );

    // Create icon button
    AppletButton *button = new AppletButton( applet, mAppletButtonGrid );

    // Position button on the grid    
    int row = (mApplets.size()-1) / MAINWINDOW_APPLETGRID_NCOL;
    int col = (mApplets.size()-1) % MAINWINDOW_APPLETGRID_NCOL;
    button->setPos( 180*col, 180*row );

    // Change left column text when applet is selected
    connect(button, SIGNAL(pressed(AppletInterface*)), this, SLOT(launchApplet(AppletInterface*)));
}

/**
 * Resize applets to make best possible use of the space avaiable
 * If description text is hidden, maximise until square
 */
void MainWindow::resizeApplets()
{
    int sz;

    QLOG_TRACE() << TAG << "before:" << mAppletRect->size();

    if( mShowText ) {
        sz = qMin( mScreen.height(), mScreen.width() - 50 - mDescriptionTextWidth );
    } else {
        sz = qMin( mScreen.height(), mScreen.width() - 50 );
    }
    mAppletRect->resize( sz, mScreen.height() - 50 );

    // Resize applets
    for(QList< AppletInterface* >::iterator it = mApplets.begin(); it != mApplets.end(); ++it)
    {
        (*it)->resize( mAppletRect->size() );
    }

    QLOG_TRACE() << TAG << "after:" << mAppletRect->size();
}

/**
 * Close the current applet
 */
void MainWindow::closeCurrentApplet()
{
    // mCurrentApplet is the current applet
    if( mCurrentApplet == NULL ) {
        //QLOG_ERROR() << TAG << "mCurrentApplet is NULL !!!";
        return;
    }

    mCurrentApplet->hide();
    mCurrentApplet->stop();
    mCurrentApplet = NULL;

    mReadyToLaunchApplet = true;
}

// Schedule closeCurrentApplet() in 0.75 sec
void MainWindow::closeCurrentAppletLater()
{
    QTimer::singleShot( 500, this, SLOT(closeCurrentApplet()) );

    mReadyToLaunchApplet = false;
}

/**
 * Start an applet
 */
void MainWindow::launchApplet(AppletInterface *applet)
{
    if( !mReadyToLaunchApplet ) return;

    mCurrentApplet = applet;

    QString text =
            "<ul align=left><li>" + applet->descriptionText() + "</li></ul>"
            "<ul align=left><li>" + applet->marketingText() + "</li></ul>"
            "<ul align=left><li>" + applet->technicalText() + "</li></ul>";

    mText->setHtml(text);

    // Accepted gesture
    QPixmap ges[3];
    applet->acceptedGestures( ges );
    mAcceptedGestures[0]->setPixmap( ges[0] );
    mAcceptedGestures[1]->setPixmap( ges[1] );
    mAcceptedGestures[2]->setPixmap( ges[2] );

    emit goApplet();
    if (mShowText) emit goAppletWithText();
    else emit goAppletWithoutText();

    // show and start
    mCurrentApplet->show();
    mCurrentApplet->start();
}

/**
 * Setup and load settings file (config.ini)
 */
void MainWindow::loadSettings()
{
    QString iniFile( QDir(qApp->applicationDirPath()).filePath( APPLICATION_SETTINGS_FILE ) );

    if( QFile::exists( iniFile ) == false )
    {
        QLOG_DEBUG() << TAG << "INI file not found at:" << iniFile;
    }

    mSettings = new QSettings( iniFile, QSettings::IniFormat );

    // Access setting values
    mShowLogger = mSettings->value("show_log", true).toBool();
    mShowBack = mSettings->value("show_back", true).toBool();
    mShowQuit = mSettings->value("show_quit", true).toBool();
    mShowText = mSettings->value("show_description_text", true).toBool();
    mCanRunWithoutMagicPad = mSettings->value("can_run_without_magic_pad", false).toBool();
    mShowInformationButton = mSettings->value("show_information_button", true).toBool();
}

/**
 *
 */
void MainWindow::keyPressEvent(QKeyEvent *event)
{
   if (event->key() == Qt::Key_Escape && mCurrentApplet != NULL) {
       emit( emulateBackButton() );
   }
}

/**
 *
 */
void MainWindow::changeText() {
   mShowText = !mShowText;
   //resizeApplets();
}
