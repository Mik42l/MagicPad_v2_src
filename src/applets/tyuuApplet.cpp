#include "tyuuApplet.h"

const QString tyuuApplet::TAG = QString("tyuuApplet");

/**
 *
 */
tyuuApplet::tyuuApplet(QWidget *parent) : Applet(parent)
{
    mName = TYUU_NAME;
    mTitle = TYUU_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();
}
