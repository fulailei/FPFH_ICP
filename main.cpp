#include "testosgQt.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
	QApplication::setAttribute(Qt::AA_DisableShaderDiskCache);
    testosgQt w;
    w.show();
	
    return a.exec();
}

