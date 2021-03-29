#ifndef BALL_H
#define BALL_H


#include <QtGui>
#include <math.h>
#include <QWidget>
#include <objectofspace.h>

class Ball : public QWidget
{
    Q_OBJECT
    
public:
    Ball(QWidget *parent = 0);
    ~Ball();
    void paintEvent(QPaintEvent *);
    void drawOfS(QPainter &paint, int xC, int yC, int R);
    int physXToWindowXCoords(double Xcoord);
    int physYToWindowYCoords(double Ycoord);
    void clear(QPainter &paint);
    void timerEvent(QTimerEvent *);
    void recountBallPositions();
    void calcFball ();

private:
    int vpw, vph;

    double ballX, ballY,  ballR;
    int i_ballX, i_ballY, i_ballR, minSizeV, idx, idy;

    int i_sqScreenSize, i_bordSize;
    int i_begEarthX, i_begEarthY, i_endEarthX, i_endEarthY;
    double  physSpaceSize, kff;
    int ikRP;   //масштаб времени
    int numbMSec; // Через сколько микросекунд

    double realTStart, realT, deltaRealT, deltaPhysT, PhysT;   // реальный отсчет времени
    double angPerSec;
    //Параметры отрисовки пушки
    int i_GunSize, i_begGunX, i_begGunY, i_endGunX, i_endGunY;
    double m_ball, f_ballX, f_ballY, v_ballX, v_ballY, a_ballX, a_ballY, dPhisT, PhysTCurr, realTNext, PhysTNext, PhysTPrev, stepPhysT;
    double angleGunGrad, angleGunRad, v_ball;

    double C_ball;      // Баллистический коэффициент
    double p_ball;      // Поправочный коэффициент
    double S_ball;      // Площадь поперечного сечения ядра
    double R_air;       // Значение сопротивления воздуха
};
typedef struct
{
    double X;
    double Y;
} vVect;

#endif // BALL_H
