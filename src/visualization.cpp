#include <QApplication>

#include "guiWindow.h"

int main(int argc, char *argv[])
{
	// General Qt app
    QApplication app(argc, argv);
    
	// GUI
	guiWindow gui;
    gui.show();

    return app.exec();
}
