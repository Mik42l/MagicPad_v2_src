#include "applet.h"

#include <QtXml>

const QString Applet::TAG = QString("Applet");

/**
 *
 */
Applet::Applet(QWidget *parent) :
    QWidget(parent)
{
    mName = "defaultName";
    mTitle = "defaultTitle";
    for (int i =0; i <  APPLET_INTERFACE_GESTURE_NUMBER; i++)
        mGestures[i] = NONE;
}

/**
 * icon - return applet icon
 */
QPixmap Applet::icon()
{
    // default icon name is ":/icon/icon_myappletname.png"
    QString fname = ":image/icon_" + mName + ".png";

    QPixmap p;
    if( !p.load( fname ) ) {
        QLOG_ERROR() << TAG << "Unable to load icon for " + mName;
    }

    return p;
}

/**
 * acceptedGestures - return accepted gestures icon in ges
 */
void Applet::acceptedGestures(QPixmap * ges)
{
    for(int i = 0; i < APPLET_INTERFACE_GESTURE_NUMBER; i++) {
        QString fname;
        switch (mGestures[i]) {
        case UP_DOWN:
            fname = ":image/icon_acc_ges_UP_DOWN.png"; break;
        case TWIST:
            fname = ":image/icon_acc_ges_TWIST.png"; break;
        case SWAP_LEFT_RIGHT:
            fname = ":image/icon_acc_ges_SWAP_LEFT_RIGHT.png"; break;
        case SWAP_BACK_FORTH:
            fname = ":image/icon_acc_ges_SWAP_BACK_FORTH.png"; break;
        case SWAP_ALL:
            fname = ":image/icon_acc_ges_SWAP_ALL.png"; break;
        case TOP_RIGHT:
            fname = ":image/icon_acc_ges_TOP_RIGHT.png"; break;
        case BOTTOM_RIGHT:
            fname = ":image/icon_acc_ges_BOTTOM_RIGHT.png"; break;
        case ALPHABET:
            fname = ":image/icon_acc_ges_ALPHABET.png"; break;
        }

        if( !ges[i].load(fname) && mGestures[i]!= NONE ) {
            QLOG_ERROR() << TAG << "Unable to load accepted gestures icon for " + mName + " number " + QString::number(i);
        }
    }
}

/**
 * setGestures - set the accepted gestures
 */
void Applet::setGestures(GestureType ges0, GestureType ges1, GestureType ges2) {
    mGestures[0] = ges0;
    mGestures[1] = ges1;
    mGestures[2] = ges2;
}

/**
 * loadTextFromXml - Load text data from config.xml
 */
void Applet::loadTextFromXml()
{
    // open config.xml
    QDomDocument doc("mydocument");
    QFile file( qApp->applicationDirPath() + "/data/text.xml" );

    if(!file.open(QIODevice::ReadOnly))
    {
        QLOG_ERROR() << TAG << "Unable to open XML text file:" << file.fileName();
    }
    if(!doc.setContent(&file))
    {
        QLOG_ERROR() << TAG << "Unable to read" << file.fileName() << "as an XML file.";
        file.close();
        return;
    }
    file.close();

    QDomElement docElem = doc.documentElement();
    QDomNodeList appletList = docElem.elementsByTagName("applet");
    for(int idx=0; idx<appletList.count(); idx++)
    {
        if(appletList.at(idx).attributes().namedItem("name").nodeValue() == mName)
        {
            //qDebug() << "   " << appletList.at(idx).toElement().tagName();
            QDomElement el = appletList.at(idx).toElement();

            QDomElement desc = el.elementsByTagName("description").item(0).toElement();
            QDomElement market = el.elementsByTagName("marketing").item(0).toElement();
            QDomElement techni = el.elementsByTagName("technical").item(0).toElement();

            if(!desc.isNull()) mDescription = desc.text();
            if(!market.isNull()) mMarketing = market.text();
            if(!techni.isNull()) mTechnical = techni.text();
        }
    }
}
