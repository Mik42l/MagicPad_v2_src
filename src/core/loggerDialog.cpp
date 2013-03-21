#include "loggerDialog.h"

/**
 * Constructor
 */
LoggerDialog::LoggerDialog(QWidget *parent)
    : QDialog(parent)
{
    // Setup GUI
    resize(640, 380);
    setWindowTitle("MagicPad message logger");
    QVBoxLayout *layout = new QVBoxLayout(this);

    mText = new QPlainTextEdit();
    mText->setReadOnly(true);

    QFont font = QFont ("Courier");
    font.setStyleHint(QFont::Monospace);
    font.setPointSize(8);
    font.setFixedPitch(true);
    mText->setFont(font);

    /*Highlighter *highlighter =*/ new Highlighter(mText->document());

    layout->addWidget(mText);

    QPushButton *closeButton = new QPushButton("&Close");
    closeButton->setDefault(true);
    connect(closeButton, SIGNAL(pressed()), this, SLOT(accept()));
    layout->addWidget(closeButton);

    mTimer = new QTimer(this);
    mTimer->setInterval(1000);
    connect(mTimer, SIGNAL(timeout()), this, SLOT(refreshText()));
    connect(this, SIGNAL(accepted()), mTimer, SLOT(stop()));
    connect(this, SIGNAL(rejected()), mTimer, SLOT(stop()));
}

/**
 * Called when the loggerDialog is showed
 */
int LoggerDialog::exec()
{
    mTimer->start();
    this->show();
    refreshText();

    return 1;
}

/**
 * Reaload the text from the logger
 */
void LoggerDialog::refreshText()
{
    //QLOG_TRACE() << "refreshing log text...";
    // TODO: log file should be DEFINED or set via logger->setLogfile(const QString& file)
    QFile file(qApp->applicationDirPath() + "/log.txt");

    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
         return;

    // Show text
    QTextStream in(&file);
    mText->setPlainText(in.readAll());

    // Scroll down
    QScrollBar *sb = mText->verticalScrollBar();
    sb->setValue(sb->maximum());
}
