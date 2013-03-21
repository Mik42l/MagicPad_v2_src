#ifndef SCOREMANAGER_H
#define SCOREMANAGER_H

#include <QObject>
#include <QGraphicsSimpleTextItem>

/**
 *
 */
class ScoreManager : public QObject
{
    Q_OBJECT

public:
    ScoreManager();
    QGraphicsSimpleTextItem* getScoreView1();
    QGraphicsSimpleTextItem* getScoreView2();
    int getScore1();
    int getScore2();

public slots:
    void updateScore1();
    void updateScore2();

private:
    int score1;
    int score2;
    QGraphicsSimpleTextItem* scoreView1;
    QGraphicsSimpleTextItem* scoreView2;
};

#endif // SCOREMANAGER_H
