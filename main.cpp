#include <QApplication>
#include "ball.h"
#include "objectofspace.h"

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    Ball w;
    w.show();
    return a.exec();

}
