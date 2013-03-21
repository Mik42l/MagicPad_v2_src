#include "scoremanager.h"
#include <QGraphicsSimpleTextItem>
#include <QBrush>
#include <QColor>
#include <QFont>
#include <QPen>


/**
 *
 */
ScoreManager::ScoreManager() :
    score1(0),
    score2(0),
    scoreView1(new QGraphicsSimpleTextItem(QString::number(0) + QString::number(0))),
    scoreView2(new QGraphicsSimpleTextItem(QString::number(00) + QString::number(0)))
{
    QBrush brush(Qt::NoBrush);
    QPen pen(QColor(85,172,50));
    QFont font("Consolas");
    font.setPointSize(18);

    scoreView1->setZValue(8);
    scoreView1->setScale(4);
    scoreView1->setPen(pen);
    scoreView1->setBrush(brush);
    scoreView1->setFont(font);

    scoreView2->setZValue(8);
    scoreView2->setScale(4);
    scoreView2->setPen(pen);
    scoreView2->setBrush(brush);
    scoreView2->setFont(font);
}

/**
 *
 */
void ScoreManager::updateScore1()
{
    score1++;
    if( score1 < 10 )
    {
        scoreView1->setText( QString::number(0) + QString::number(score1) );
    } else {
        scoreView1->setText( QString::number(score1) );
    }
}

/**
 *
 */
void ScoreManager::updateScore2()
{
    score2++;
    if(score2<10)
    {
        scoreView2->setText( QString::number(0) + QString::number(score2) );
    } else {
        scoreView2->setText( QString::number(score2) );
    }
}

/**
 *
 */
QGraphicsSimpleTextItem* ScoreManager::getScoreView1()
{
    return scoreView1;
}

/**
 *
 */
QGraphicsSimpleTextItem* ScoreManager::getScoreView2()
{
    return scoreView2;
}

/**
 *
 */
int ScoreManager::getScore1()
{
    return score1;
}

/**
 *
 */
int ScoreManager::getScore2()
{
    return score2;
}
