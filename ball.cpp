#include "ball.h"
#include <QPair>
#include <QPainter>
#include <time.h>
#include "objectofspace.h"
#include <stdio.h>
#include <cmath>
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define G_VAL 9.8
#define FL_SOPR 0
#define NUMBSP 12
#define PI_VAL 3.14159265359
#define RadEarth 6371000
#define RadMoon  1737100
#define MSun  1.9885e30
#define MEarth  5.97e24
#define aOne 149597870700
#define RSun 6.9551e8
#define NPlanet 10


double G_Newton, RPlanet, DPlanet, DAngPlanet, angPlanet;

ObjectOfSpace OfS[NUMBSP];

Ball::~Ball()
{
}
Ball::Ball(QWidget *parent)
    : QWidget(parent)
{
    QPainter paint(this);
    G_Newton = 6.6738480e-11;
    double h1Earth = 300000+397403000-OfS[0].x, //высота орбиты первой ракеты
    angle1Grad = 0.0,
    vRocket1 = 1500,
    angle1Rad = angle1Grad*PI_VAL/180.0;
    double h2Earth = 300000, //высота орбиты первой ракеты
    angle2Grad = 231.5,
    vRocket2 = 10850,
    angle2Rad = angle2Grad*PI_VAL/180.0;
    //сириус а. берем нуль в точке центра масс системы
    OfS[0].m = MSun*2.14;
    OfS[0].x = -0.31*aOne*37;
    OfS[0].y = 0.0;
    OfS[0].vX = 0;
    OfS[0].vY = -1100;
    OfS[0].R = 5;
    OfS[0].colorObj = Qt::darkGray;
    OfS[0].typeObj = 0; // 0-планета, 1-спутник\ракета
    OfS[0].freeObj = 1;
    OfS[0].physR = 1.7*RSun;

    //сириус B
    OfS[1].m = MSun*0.98;
    OfS[1].x = 0.69*aOne*37;
    OfS[1].y = 0.0;
    OfS[1].vX = 0;
    OfS[1].vY = 2400;
    OfS[1].R = 3;
    OfS[1].colorObj = Qt::darkGray;
    OfS[1].typeObj = 0;
    OfS[1].freeObj = 1;
    OfS[1].physR = RadEarth;


    RPlanet = 30*aOne; //расстояние от центра масс
    DAngPlanet = 1.3*PI_VAL;
    double VPlanet;
    DPlanet = 0;//начальное расстояние между орбитами планет


    for (int i=2; i<NPlanet+2; i++){
       angPlanet = (i-2)*DAngPlanet;
       OfS[i].m = MEarth;
       OfS[i].x = (RPlanet+DPlanet)*cos(angPlanet);
       OfS[i].y = (RPlanet+DPlanet)*sin(angPlanet);
       VPlanet = sqrt(G_Newton*(OfS[0].m+OfS[1].m)/(RPlanet+DPlanet));

       OfS[i].vX = VPlanet *cos(angPlanet+PI_VAL/2);
       OfS[i].vY = VPlanet *sin(angPlanet+PI_VAL/2);

       OfS[i].R = 2;
       OfS[i].colorObj = Qt::red;
       OfS[i].typeObj = 1;
       OfS[i].freeObj = 1;
       OfS[i].physR = 3;
       DPlanet += 6*aOne;
    }
    OfS[10].colorObj = Qt::green;
    OfS[11].colorObj = Qt::blue;

    for (int i=0; i<NUMBSP; i++){
        OfS[i].aX = 0;
        OfS[i].aY = 0;
    }

    resize(1920,1080);
    // Ширина и высота открытого окна
    vpw = width();
    vph = height();
    minSizeV = MIN(vpw, vph);   // Минимальная сторона прямоугольника-окна
    i_ballR = 8;       //Радиус рисования ядра в окне экрана (виджета)
    idx = (vpw-minSizeV)/2;
    idy = (vph-minSizeV)/2;
    i_sqScreenSize = minSizeV;  // Размер стороны окна, в которое мы отображаем квадрат физического пр-ва

    // Параметры отрисовки линии земли
    i_begEarthX = 0;
    i_begEarthY = vph - (idy + (i_bordSize-i_ballR));
    i_endEarthX = vpw;
    i_endEarthY = i_begEarthY;

    // Физическая модель
    m_ball = 50;    //масса ядра
    ballR = 0.1;      // Радиус ядра
    C_ball = 0.47;      // Баллистический коэффициент
    p_ball = 1.225;      // Поправочный коэффициент
    S_ball = M_PI*ballR*ballR;   // Площадь поперечного сечения ядра

    // Начальные координаты ядра
    ballX = 0.0;
    ballY = 0.0;

    // Начальная скорость ядра
    v_ball = 330.0;       // Просто скорость вылета снаряда из пушки
    angleGunGrad = 45.0; // Угол в градусах
    angleGunRad = angleGunGrad*M_PI/180.0;  // Перевели в радианы
    v_ballX = v_ball*cos(angleGunRad);
    v_ballY = v_ball*sin(angleGunRad);

    calcFball();    // Вычислим параметры силы воздействия на ядро сразу после выстрела

    // Начальное ускорения ядра
    a_ballX = f_ballX/m_ball;
    a_ballY = f_ballY/m_ball;

    physSpaceSize = 500.0*aOne;    // Размер отображаемого квадрата в физическом пространстве (в м)

    // Вычисляем параметры пересчета физических координат в экранные
    kff = double(i_sqScreenSize)/(physSpaceSize);

    //Преобразуем начальные
    i_ballX = physXToWindowXCoords(ballX);
    i_ballY = physYToWindowYCoords(ballY);

    //Параметры отрисовки пушки
    i_GunSize = i_bordSize;  // Размер пушки не должен превышать ширину бордюра
    i_begGunX = i_ballX;   //Один конец совпадает
    i_begGunY = i_ballY;   //  с точкой вылета ядра
    i_endGunX = i_ballX - double (i_GunSize)*cos(angleGunRad);
    i_endGunY = i_ballY + double (i_GunSize)*sin(angleGunRad);

    numbMSec = 20;    // Через скольуо микросекунд прерывания
    startTimer(numbMSec);    //  Запускаем регулярное прерывание таймером, чтобы перерисовывать 50 раз в секунду
    deltaRealT = 0.001*numbMSec;   // Приращение реального времени
    realTStart = double(clock())/CLOCKS_PER_SEC; // Начальное время в секундах
    PhysT = 0.0;
    PhysTPrev = 0.0;

    ikRP = (int)((50*365*24*3600)/5); //масштаб времени
    deltaPhysT = 10000;
}

// перевод физических координат в экранные
int Ball::physXToWindowXCoords(double Xcoord)
{
    return(kff*(Xcoord)+idx+i_sqScreenSize/2);
}
int Ball::physYToWindowYCoords(double Ycoord)
{
    return(kff*(physSpaceSize-Ycoord)+idy-i_sqScreenSize/2);
}

// Расчет вектора силы, действующей на ядро
void Ball::calcFball()
{
    double lenV2;       //Квадрат скорости
    double lenV;        //значение скорости
    double ev_x, ev_y;  // единичный вектор

    lenV2 = v_ballX*v_ballX + v_ballY*v_ballY;
    R_air = C_ball*p_ball*lenV2*S_ball/2;
    lenV = sqrt(lenV2);
    // единичный вектор, колинеарный со скоростью
    ev_x = v_ballX/lenV;
    ev_y = v_ballY/lenV;

    f_ballX = FL_SOPR*(-R_air*ev_x);
    f_ballY = FL_SOPR*(-R_air*ev_y)-G_VAL*m_ball;
}

// Пересчет положения космических тел методом Гюна
void Ball::recountBallPositions()
{


    PhysT = PhysTPrev + deltaPhysT;
    while (PhysT <= PhysTCurr)
    {  for (int i=0; i<NUMBSP; i++){
           OfS[i].xNext = OfS[i].x + deltaPhysT*OfS[i].vX;
           OfS[i].yNext = OfS[i].y + deltaPhysT*OfS[i].vY;
           OfS[i].xVNext = OfS[i].vX + deltaPhysT*OfS[i].aX;
           OfS[i].yVNext = OfS[i].vY + deltaPhysT*OfS[i].aY;

           //Вычислим силу, действующую на планету i
           OfS[i].xF = 0.0;
           OfS[i].yF = 0.0;
           double eX, eY, s_q;
           for (int j=0; j<NUMBSP; j++){
               if (i!=j){
                  s_q = sqrt((OfS[j].x-OfS[i].x)*(OfS[j].x-OfS[i].x)+(OfS[j].y-OfS[i].y)*(OfS[j].y-OfS[i].y));
                  eX = (OfS[j].x-OfS[i].x)/s_q;
                  eY = (OfS[j].y-OfS[i].y)/s_q;
                  OfS[i].xF +=G_Newton*eX*(OfS[i].m*OfS[j].m)/(s_q*s_q);
                  OfS[i].yF +=G_Newton*eY*(OfS[i].m*OfS[j].m)/(s_q*s_q);
               }
           }
           // Вычислим ускорение космического тела
           OfS[i].xANext = OfS[i].xF/ OfS[i].m;
           OfS[i].yANext = OfS[i].yF/ OfS[i].m;

           OfS[i].x = OfS[i].x + deltaPhysT*(OfS[i].vX+OfS[i].xVNext)/2;
           OfS[i].y = OfS[i].y + deltaPhysT*(OfS[i].vY+OfS[i].yVNext)/2;
           OfS[i].vX = OfS[i].vX + deltaPhysT*(OfS[i].aX+OfS[i].xANext)/2;
           OfS[i].vY = OfS[i].vY + deltaPhysT*(OfS[i].aY+OfS[i].yANext)/2;
           // проверка для ракет падения на планеты
           if (OfS[i].typeObj == 1 && OfS[i].freeObj == 1) {
               for (int k=0; k<NUMBSP; k++){
                   if (OfS[k].typeObj == 0){
                       if ((OfS[i].x-OfS[k].x)*(OfS[i].x-OfS[k].x)+(OfS[i].y-OfS[k].y)*(OfS[i].y-OfS[k].y) <= (OfS[k].physR)*(OfS[k].physR))
                              OfS[i].freeObj = 0;

                   }

               }
           }

        }

        //Вычислим ускорение для вновь вычисленных положений планет
        for (int i=0; i<NUMBSP; i++){
            OfS[i].xF = 0.0;
            OfS[i].yF = 0.0;
            double eX, eY, s_q;
            for (int j=0; j<NUMBSP; j++){
                if (i!=j){
                   s_q = sqrt((OfS[j].x-OfS[i].x)*(OfS[j].x-OfS[i].x)+(OfS[j].y-OfS[i].y)*(OfS[j].y-OfS[i].y));
                   eX = (OfS[j].x-OfS[i].x)/s_q;
                   eY = (OfS[j].y-OfS[i].y)/s_q;
                   OfS[i].xF +=G_Newton*eX*(OfS[i].m*OfS[j].m)/(s_q*s_q);
                   OfS[i].yF +=G_Newton*eY*(OfS[i].m*OfS[j].m)/(s_q*s_q);
                }
                // Вычислим ускорение космического тела
                OfS[i].aX = OfS[i].xF/ OfS[i].m;
                OfS[i].aY = OfS[i].yF/ OfS[i].m;
            }


        }
        PhysTPrev = PhysT;      //Запомним этот момент времени как предыдущий
        PhysT = PhysT+deltaPhysT;  // Вычислим следующий момент времени



    }
}

void Ball::timerEvent(QTimerEvent *)
{
    realT = double(clock())/CLOCKS_PER_SEC - realTStart; // Реальное время в секундах от начала движения снаряда
    PhysTCurr = realT*ikRP;   // Текущее физическое время от начала движения снаряда
    recountBallPositions();

    update();
}

void Ball::paintEvent(QPaintEvent *) // Головной блок воспроизведения графики
{
    QPointF cP;
    QPainter paint(this);

    // Изменение очереди для хвоста
    double xD, yD;
    int xC, yC, day, years;
    paint.setPen(QPen(Qt::darkCyan,1,Qt::SolidLine));
    paint.setBrush (QBrush(Qt::white, Qt::SolidPattern));
    paint.drawRect(0,0,vpw, vph);
    for (int i=0; i<NUMBSP; i++){  // Преобразуем физические координаты ядра в оконные

        xC = physXToWindowXCoords(OfS[i].x);
        yC = physYToWindowYCoords(OfS[i].y);

        if (OfS[i].freeObj == 1) {
            paint.setBrush (QBrush(OfS[i].colorObj, Qt::SolidPattern));
            paint.setPen(QPen(OfS[i].colorObj,1,Qt::SolidLine));
            drawOfS(paint, xC, yC, OfS[i].R);
        }
        cP.setX(OfS[i].x);
        cP.setY(OfS[i].y);
        if (OfS[i].qnObj->statusQueue() == false || OfS[i].freeObj == 0) OfS[i].qnObj->popPoint();
        if (OfS[i].freeObj == 1) OfS[i].qnObj->pushPoint(cP);

        // рисование хвоста
        paint.setPen(QPen(OfS[i].colorObj,1,Qt::SolidLine));
        for (int j=0; j<OfS[i].qnObj->getNumbPoint(); j++)
        {
            cP = OfS[i].qnObj->getPointN(j);
            xD = (double)cP.x();
            yD = (double)cP.y();
            xC = physXToWindowXCoords(xD);
            yC = physYToWindowYCoords(yD);
            paint.drawPoint(xC, yC);
        }

    }
    years = (int)(PhysT/(86400*365));
    char buf[40];
    paint.setPen(QPen(Qt::black,1,Qt::SolidLine));
    sprintf(buf,"%i",years);

    paint.drawText (10, 20,60, 20, Qt::AlignLeft | Qt::AlignVCenter,"Years");
    paint.drawText (70, 20,60, 20, Qt::AlignLeft | Qt::AlignVCenter, buf);



}

void Ball::drawOfS(QPainter &paint, int xC, int yC, int R)

{
    double dist;
    paint.drawEllipse(xC-R,yC-R,2*R,2*R);
}

void Ball::clear(QPainter &paint)
{
    paint.eraseRect(0,0,200,200);
}





