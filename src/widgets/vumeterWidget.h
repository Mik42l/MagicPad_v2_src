#ifndef VUMETER_H
#define VUMETER_H

#include <QWidget>
#include <QPainter>
#include <QVector>

class VuMeterWidget : public QWidget
{
    Q_OBJECT
public:

    static const QString TAG;

    VuMeterWidget(QWidget *parent = 0);
    void draw(QPainter *painter);
    void setRange(int _min, int _max);
    void setLevelNumber(int n);
signals:

protected:
    void paintEvent(QPaintEvent *event);

public slots:
    void setValue(int v);

private:
    int m_value;
    int m_min;
    int m_max;

    // Number of levels (== steps)
    int m_levels;
    QVector<int> m_colormap;

};

#endif // VUMETER_H
