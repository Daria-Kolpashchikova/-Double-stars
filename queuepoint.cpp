#include "queuepoint.h"
#include "ball.h"
#include <QPair>

QueuePoint::~QueuePoint()
{
}
QueuePoint::QueuePoint(int maxQueue)

{ qNumb = 0;
  qSize = maxQueue;
  qHead = -1;
  qLast = 0;
}

bool QueuePoint::pushPoint(QPointF currPoint)
{
  if (qNumb >= qSize)
      return false;
  qNumb++;
  qHead++;
  if (qHead >= qSize) qHead = 0;
  qP [qHead] = currPoint;
  return true;
}

bool QueuePoint:: popPoint(void){
    if (qNumb == 0) return false;
    qLast++;
    qNumb--;
    if (qLast >= qSize) qLast = 0;
}

int QueuePoint::getNumbPoint (void){
    return qNumb;
}
bool QueuePoint::statusQueue (void){
    if (qNumb<qSize)  return true;
    return false;
}

QPointF QueuePoint::getPointN (int nQueue){
    int j;
    j = qHead-nQueue;
    if (j>=0) return qP[j];
    return qP[qSize+j];
}



