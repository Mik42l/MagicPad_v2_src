#include "pandaApplet.h"

pandaApplet::pandaApplet(QWidget *parent) : Applet(parent)
{
    mName = PANDA_NAME;
    mTitle = PANDA_TITLE;

    // load description, marketing and technical text from
    // xml file. Ensure that mName is set before calling this function
    loadTextFromXml();
}
