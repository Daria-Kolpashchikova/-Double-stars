#ifndef QUEUEPOINT_H
#define QUEUEPOINT_H
#include <QObject>
#include <QPointF>

class QueuePoint
{

public:
    QueuePoint (int maxQueue);
    ~QueuePoint ();

    bool pushPoint(QPointF currPoint);
    bool popPoint(void);
    int getNumbPoint (void);
    QPointF getPointN (int nQueue);
    bool statusQueue (void);


private:
    QPointF qP [1000];
    int qHead, qLast, qNumb, qSize;
};
#endif // QUEUEPOINT_H
