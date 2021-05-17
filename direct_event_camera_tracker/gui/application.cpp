#include "application.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// see eg. https://stackoverflow.com/a/4661911

bool MyApp::notify(QObject* receiver, QEvent* event)
{
    QString err;
    try {
        return QApplication::notify(receiver, event);
    } catch (std::exception &e) {
        err = e.what();
    } catch (const char* e) {
        err = e;
    } catch (...) {
        err = "UNKNOWN";
    }


    QString event_type(typeid(*event).name());
    QString receiver_name(qPrintable(receiver->objectName()));
    QString receiver_type(typeid(*event).name());


    cerr << "An exception was thrown" << endl << endl;
    cerr << err.toStdString() << endl << endl;
    cerr << "When sending event " << event_type.toStdString() << " to " << receiver_name.toStdString();
    cerr << " (" << receiver_type.toStdString() << ")" << endl << endl;
    cerr << "Application might be in an undefined state now." << endl;

    // this is a bit fishy...
    QMessageBox::critical(nullptr, "Exception",
        QString("An exception was thrown:<br><br>")
        + "<big><b>" + err + "</b></big><br><br>"
        + "When sending event " + event_type + " to object <b>"  + receiver_name
        + "</b> (" + receiver_type + ")<br><br>"
        + "Application might be in an undefined state now.");

    return true;
}

////////////////////////////////////////////////////////////////////////////////

