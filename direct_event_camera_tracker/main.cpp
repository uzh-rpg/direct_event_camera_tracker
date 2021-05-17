#include "main.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

////////////////////////////////////////////////////////////////////////////////

int main_gui(int argc, char** argv)
{
    MyApp app(argc, argv);

    QCoreApplication::setOrganizationName("UZH RPG");
    QCoreApplication::setApplicationName("direct event tracker");
    QCoreApplication::setApplicationVersion("0.0.1");

    // request OpenGL 3.2
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setOption(QSurfaceFormat::DebugContext); // only for debugging!
    format.setSamples(8); // enable anti-aliasing (multisampling)
    QSurfaceFormat::setDefaultFormat(format);

    cout << "reading config from '" << argv[1] << "'" << endl;
    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (const YAML::ParserException& e) {
        cerr << "ERROR: Failed to parse YAML: " << e.what() << endl;
        return -1;
    } catch (const YAML::BadFile& e) {
        cerr << "ERROR: Failed to read config file: Bad file. Does it exist?" << endl;
        cerr << "       Current working directory: " << QDir::currentPath().toStdString() << endl;
        return -1;
    }

    MainGUI maingui(config);
    maingui.show();

    return app.exec();
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    google::InitGoogleLogging(argv[0]);
    ros::start();

    cout << "Qt Version: " << QT_VERSION_STR << endl;

    try {
        if (argc > 1) {
            return main_gui(argc, argv);
        } else {
            cerr << "please provide path to a config.yaml as first parameter (e.g. 'cfg/main.yaml')" << endl;
        }
    } catch (const std::exception& e) {
        cerr << "ERROR: " << e.what() << endl;
    } catch (const char* e) {
        cerr << "ERROR: " << e << endl;
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
