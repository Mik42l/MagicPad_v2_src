#ifndef OVENTIMER_H
#define OVENTIMER_H

#include <QtCore/qmath.h>
#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QWidget>

#define OVENTIMER_DEGREE_PER_MINUTE 7.0
#define OVENTIMER_DEGREE_PER_SECOND (OVENTIMER_DEGREE_PER_MINUTE / 60.0)
#define OVENTIMER_MAX_MINUTES 45
#define OVENTIMER_MAX_SECONDS (OVENTIMER_MAX_MINUTES * 60)
#define OVENTIMER_UPDATE_INTERVAL 1.0

/**
 *
 */
class OvenTimerWidget : public QWidget
{
    Q_OBJECT
	
public:

	const static QString TAG;

    OvenTimerWidget(QWidget *parent = 0);

    int duration() const;

    void draw(QPainter *painter);

    void setRange(int _min, int _max) {
        m_min = _min;
        m_max = _max;
    }

public slots:

    void setDuration(int secs);

    void setValue(int v);

signals:

    void timeout();

protected:

    void paintEvent(QPaintEvent *event);

    void mousePressEvent(QMouseEvent *event);

private:

    QDateTime finishTime;
    QTimer *updateTimer;
    QTimer *finishTimer;
    int m_min;
    int m_max;
};

#endif // OVENTIMER_H
