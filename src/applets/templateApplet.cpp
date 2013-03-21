#include "templateApplet.h"

TemplateApplet::TemplateApplet(QWidget *parent) : Applet(parent)
{
    mName = TEMPLATEAPPLET_NAME;
    mTitle = TEMPLATEAPPLET_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();
}
