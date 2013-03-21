#ifndef ALPHABETAPPLET_H
#define ALPHABETAPPLET_H

#include "applet.h"

#include <QTimer>
#include <QSignalTransition>
#include <QAbstractTransition>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QStateMachine>
#include <QTime>
#include <QVBoxLayout>

#include "filters/calibrationFilter.h"
#include "filters/alphabetFilter.h"

#define ALPHABET_NAME "alphabet"
#define ALPHABET_TITLE "Alphabet"

// TODO: QWidget instead of QGraphicScene

/**
 * Pixmap is a letter image
 */
class Pixmap : public QObject, public QGraphicsPixmapItem
{

    Q_OBJECT
    Q_PROPERTY(QPointF pos READ pos WRITE setPos)
    Q_PROPERTY(qreal scale READ scale WRITE setScale)

public:

    Pixmap(const QPixmap &pix)
        : QObject(), QGraphicsPixmapItem(pix)
    {
        //setCacheMode(DeviceCoordinateCache);
    }
};

/**
 * Main scene
 */
class AlphabetView : public QGraphicsView
{

public:

    AlphabetView(QGraphicsScene *scene) : QGraphicsView(scene) { }

protected:

    void resizeEvent(QResizeEvent *event)
    {
        QGraphicsView::resizeEvent(event);
        fitInView(sceneRect(), Qt::KeepAspectRatio);
    }
};

/**
 * Alphabet applet
 */
class AlphabetApplet : public Applet
{

    Q_OBJECT

public:

    static const QString TAG;

    AlphabetApplet(QWidget *parent = 0);

public slots:

    void setFrame( cv::Mat& frame );

signals:

    void trigger_A();
    void trigger_G();
    void trigger_H();
    void trigger_I();
    void trigger_K();
    void trigger_L();
    void trigger_O();
    void trigger_R();
    void trigger_S();
    void trigger_T();
    void trigger_W();
    void trigger_unknow();

private:

    CalibrationFilter *mCalibrationFilter;
    AlphabetFilter *mAlphabetFilter;

    QPixmap bgPix;
    QList<Pixmap *> pixmaps;
    QList<Pixmap *> items;

    QGraphicsScene *scene;
    AlphabetView *view;

    unsigned char mLastChar;
    QStateMachine states;
    QState *rootState;
    QState *ellipseState;
    QState *centeredState;
    QList<QState *> statesLetters;
    QParallelAnimationGroup *group;
    QAbstractTransition *trans;
    QTimer *mStartTimer;
};

#endif // ALPHABETAPPLET_H
