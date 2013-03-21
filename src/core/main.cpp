
#include <QApplication>
#include <QMessageBox>
#include <QPixmap>
#include <QSplashScreen>
#include <QFile>

#include "QsLog.h"
#include "QsLogDest.h"

#include "mainwindow.h"
#include "version.h"

// Log file
#define APPLICATION_LOG_FILE "/log.txt"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QFile file(qApp->applicationDirPath() + "/log.txt");

    if(!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
        QMessageBox m;
        m.setText( "Error opening " + file.fileName() + " with error #" + QString::number( file.error() ) );
        m.exec();
    }

    // Splashscreen
    QPixmap pixmap(":/image/isorg.jpg");
    QSplashScreen splash( pixmap );
    splash.show();
    splash.showMessage( "Debug information" );

    // init the logging mechanism
    QsLogging::Logger& logger = QsLogging::Logger::instance();

    logger.setLoggingLevel(QsLogging::TraceLevel);

    const QString sLogPath( qApp->applicationDirPath() + APPLICATION_LOG_FILE );

    QsLogging::DestinationPtr fileDestination(
        QsLogging::DestinationFactory::MakeFileDestination(sLogPath) );

    QsLogging::DestinationPtr debugDestination(
        QsLogging::DestinationFactory::MakeDebugOutputDestination() );

    logger.addDestination(debugDestination.get());
    logger.addDestination(fileDestination.get());

    // Print program info: Version, Start time, ...
    QLOG_INFO() << "------------------------------------------------------------";
    QLOG_INFO() << QDateTime::currentDateTime().toString("yyyy-MM-dd @ hh:mm:ss");

    QLOG_INFO() << "MagicPad version : " << MAGICPAD_VERSION_STR;
    QLOG_INFO() << "QT version : " << QT_VERSION_STR;
    QLOG_INFO() << "OpenCV version : " << OPENCV_VERSION;

    splash.showMessage( "Building main application" );
    MainWindow w;
    w.show();

    // Hide splashscreen
    splash.finish( &w );
    
    return a.exec();
}

