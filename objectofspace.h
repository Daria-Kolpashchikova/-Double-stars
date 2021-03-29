#pragma once
#ifndef OBJECTOFSPACE_H
#define OBJECTOFSPACE_H
#include "queuepoint.h"
#include <QPainter>

#include <QObject>

#define LENQUENE 900

class ObjectOfSpace
{

public:
    ObjectOfSpace();
    ~ObjectOfSpace();
    double x, y, vX, vY, aX,  aY, m, R, physR, xF, yF, yNext, xNext, yVNext, xVNext, xANext, yANext;
    int typeObj; // 0-планета, 1-спутник/ракета
    bool freeObj; // 1-летит 0-упал
    QColor colorObj;
    QueuePoint *qnObj;
};

#endif // OBJECTOFSPACE_H
