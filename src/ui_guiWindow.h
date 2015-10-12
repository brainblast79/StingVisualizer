/********************************************************************************
** Form generated from reading UI file 'guiWindowHp8264.ui'
**
** Created: Fri 9. Jan 09:56:36 2015
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef GUIWINDOWHP8264_H
#define GUIWINDOWHP8264_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_guiWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QVTKWidget *qvtkWidget;
    QGroupBox *Controls_Group;
    QHBoxLayout *horizontalLayout;
    QPushButton *Start_button;
    QPushButton *Pause_button;
    QPushButton *Stop_button;
    QLabel *Log_File_Label;
    QLineEdit *Sensor_File_Save_Box;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *guiWindow)
    {
        if (guiWindow->objectName().isEmpty())
            guiWindow->setObjectName(QString::fromUtf8("guiWindow"));
        guiWindow->setEnabled(true);
        guiWindow->resize(933, 539);
        centralwidget = new QWidget(guiWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        centralwidget->setMaximumSize(QSize(16777215, 548));
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));

        gridLayout->addWidget(qvtkWidget, 2, 0, 1, 1);

        Controls_Group = new QGroupBox(centralwidget);
        Controls_Group->setObjectName(QString::fromUtf8("Controls_Group"));
        Controls_Group->setMaximumSize(QSize(16777215, 80));
        horizontalLayout = new QHBoxLayout(Controls_Group);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Start_button = new QPushButton(Controls_Group);
        Start_button->setObjectName(QString::fromUtf8("Start_button"));

        horizontalLayout->addWidget(Start_button);

        Pause_button = new QPushButton(Controls_Group);
        Pause_button->setObjectName(QString::fromUtf8("Pause_button"));

        horizontalLayout->addWidget(Pause_button);

        Stop_button = new QPushButton(Controls_Group);
        Stop_button->setObjectName(QString::fromUtf8("Stop_button"));

        horizontalLayout->addWidget(Stop_button);

        Log_File_Label = new QLabel(Controls_Group);
        Log_File_Label->setObjectName(QString::fromUtf8("Log_File_Label"));

        horizontalLayout->addWidget(Log_File_Label);

        Sensor_File_Save_Box = new QLineEdit(Controls_Group);
        Sensor_File_Save_Box->setObjectName(QString::fromUtf8("Sensor_File_Save_Box"));

        horizontalLayout->addWidget(Sensor_File_Save_Box);


        gridLayout->addWidget(Controls_Group, 1, 0, 1, 1);

        guiWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(guiWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        guiWindow->setStatusBar(statusbar);
#ifndef QT_NO_SHORTCUT
        Log_File_Label->setBuddy(Sensor_File_Save_Box);
#endif // QT_NO_SHORTCUT

        retranslateUi(guiWindow);

        QMetaObject::connectSlotsByName(guiWindow);
    } // setupUi

    void retranslateUi(QMainWindow *guiWindow)
    {
        guiWindow->setWindowTitle(QApplication::translate("guiWindow", "Path Controller", 0, QApplication::UnicodeUTF8));
        Controls_Group->setTitle(QApplication::translate("guiWindow", "Controls", 0, QApplication::UnicodeUTF8));
        Start_button->setText(QApplication::translate("guiWindow", "Start", 0, QApplication::UnicodeUTF8));
        Pause_button->setText(QApplication::translate("guiWindow", "Pause OFF", 0, QApplication::UnicodeUTF8));
        Stop_button->setText(QApplication::translate("guiWindow", "Stop", 0, QApplication::UnicodeUTF8));
        Log_File_Label->setText(QApplication::translate("guiWindow", "File Prefix", 0, QApplication::UnicodeUTF8));
        Sensor_File_Save_Box->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class guiWindow: public Ui_guiWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // GUIWINDOWHP8264_H
