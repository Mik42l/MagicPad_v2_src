#ifndef DESCRIPTIONTEXT_H
#define DESCRIPTIONTEXT_H

#include "QsLog.h"
#include <QLabel>

/**
 *
 */
class DescriptionText : public QLabel
{

    Q_OBJECT

public:

    DescriptionText(QWidget *parent = 0);
	
    // Set the description text
    void setDescriptionText( const QString& description,
				  const QString& marketing,
				  const QString& technical );

private:

};

#endif // DESCRIPTIONTEXT_H
