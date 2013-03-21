#ifndef IAMANAGER_H
#define IAMANAGER_H

#include "ball.h"
#include "barIA.h"

/**
 *
 */
class IAManager
{

public:

    IAManager( BarIA* newBarIA);
    void actualizeBarIADirection(const QList<Ball*> & ballCollection);

private:
    BarIA* barIA;

};

#endif // IAMANAGER_H
