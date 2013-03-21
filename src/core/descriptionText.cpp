#include "descriptionText.h"

/**
 *
 */
DescriptionText::DescriptionText(QWidget *parent) : QLabel(parent)
{
    setAlignment( Qt::AlignTop );

    // Set Font
    QFont f = font();
    f.setPointSize( 14 );
    setFont( f );

    QPalette p = palette();
    p.setBrush( QPalette::Active, QPalette::WindowText, Qt::white );
    setPalette( p );

    //setGeometry( 50, 50, 400, 800 );
    setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );
    setFixedWidth( 400 );
    setWordWrap( true );
}

/**
 *
 */
void DescriptionText::setDescriptionText(
    const QString& description,
    const QString& marketing,
    const QString& technical )
{
    setText( "<html><ul align=left><li>" + description + "</li></ul>"
        "<ul align=left><li>" + marketing + "</li></ul>"
        "<ul align=left><li>" + technical + "</li></ul></html>" );

    adjustSize();
}
