#ifndef APPLICATION_H_SXVFAQYY
#define APPLICATION_H_SXVFAQYY

#include <QApplication>
#include <QMessageBox>
#include <exception>
#include <QEvent>
#include <QObject>
#include <typeinfo>
#include <iostream>

// overwrite default application to handle generic exceptions

class MyApp : public QApplication
{
public:
    MyApp(int &argc, char **argv) : QApplication(argc, argv) { std::cout << "created MyApp" << std::endl;};

    bool notify(QObject* receiver, QEvent* event);
};


#endif /* end of include guard: APPLICATION_H_SXVFAQYY */
