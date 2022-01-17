#include "maingui.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;

////////////////////////////////////////////////////////////////////////////////

MainGUI::MainGUI(const YAML::Node& config)
    : QMainWindow(),
    config(config),
    jacobi_tester(nullptr),
    evo_map_publisher(nullptr),
    ros_spin_timer(this),
    pyramid(config),
    plot_matrix(new PlotMatrix(*this)),
    last_published_pose_marker_id(0)
{
    std::cout << "Eigen Version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    ui.setupUi(this);

    ui.opt_param_pane->setFloating(true);

    ui.tracking_timestep_ms->setValue(config["tracking"]["time_step"].as<double>());

    if (config["initial_pose"]) {
        ui.selected_pose->setState(config["initial_pose"].as<Statef>());
    }

    pyramid.link_table(*ui.image_pyramid);

    QSettings settings;
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState   (settings.value("windowState").toByteArray());
    ui.splitter->restoreGeometry(settings.value("splitter/geometry").toByteArray());

    //ui.legend_grad->setPixmap(plot_gradient_legend());

    // show legend for graphs that plot vector fields
    ui.sel_pose_gradient->drawAngularBorder(4);
    ui.sel_pose_flow->drawAngularBorder(4);
    ui.kf_gradient->drawAngularBorder(4);
    ui.kf_flow->drawAngularBorder(4);

    ui.current_pyramid_level->setMaximum(pyramid.get_size());

    rospub_hover_pose     = node.advertise<geometry_msgs::PoseStamped>("/pose/hover", 100);
    rospub_current_pose   = node.advertise<geometry_msgs::PoseStamped>("/pose/current", 100);
    rospub_keyframe_pose  = node.advertise<geometry_msgs::PoseStamped>("/pose/keyframe", 100);
    rospub_gt_track       = node.advertise<visualization_msgs::Marker>("/tracking/gt", 100);
    rospub_current_pose_marker = node.advertise<visualization_msgs::Marker>("/tracking/pose/current", 100);
    rospub_map            = node.advertise<sensor_msgs::PointCloud2>("/map/cloud", 1);

    if (config["bagfile"]) {
        cout << "loading bagfile '" << config["bagfile"] << "'\n";
        load_bag(config["bagfile"].as<std::string>());
    } else {
        throw "No input 'bagfile' specified in config!";
    }

    if (config["camera"]) {
        if (config["camera"]["calibration"]) {
            // warning: this *overwrites* existing config!
            data.load_camera_config(config["camera"]["calibration"].as<string>());
        }

        if (config["camera"]["hand_eye"]) {
            data.load_hand_eye(config["camera"]["hand_eye"].as<string>());
        }

        data.getCamera().setClipping(
                config["camera"]["near"].as<double>(),
                config["camera"]["far"].as<double>());
    } else {
        cerr << "WARNING: No camera clipping parameters provided in config." << endl;
    }

    data.event_count_density = config["tracking"]["event_count_density"].as<double>();
    ui.event_density->setValue(data.event_count_density);

    world_renderer = new WorldRenderer(this, data.getCamera());
    ui.keyframe_layout->addWidget(world_renderer, 4, 1);

    if (config["map"]) {

        MapType map_type = MapType::AUTO;
        if (config["map_type"]) {
            if (config["map_type"].as<string>() == "cloud") {
                map_type = MapType::CLOUD;
            } else if (config["map_type"].as<string>() == "mesh") {
                map_type = MapType::AUTOSHADE_MESH;

                if (config["mesh_shading"]) {
                    if (config["mesh_shading"].as<bool>()) {
                        cout << "explicitely enabling shading" << endl;
                        map_type = MapType::SHADED_MESH;
                    } else {
                        cout << "explicitely disabling shading" << endl;
                        map_type = MapType::UNSHADED_MESH;
                    }
                }

            } else if (config["map_type"].as<string>() == "auto") {
                map_type = MapType::AUTO;
            } else {
                throw "Invalid map type. Only 'cloud', 'mesh' or 'auto' is allowed.";
            }
        }


        world_renderer->loadMap(config["map"].as<string>(), map_type);

    } else {
        throw "Now 'map' defined in config!";
    }

    connect(ui.err_plot_lbl, &ErrplotLabel::stateSelected, this, &MainGUI::select_pose);
    connect(ui.err_plot_lbl, &ErrplotLabel::stateHovered,  this, &MainGUI::hover_pose);

    connect(plot_matrix, &PlotMatrix::stateSelected, this, &MainGUI::select_pose);
    connect(plot_matrix, &PlotMatrix::stateHovered,  this, &MainGUI::hover_pose);

    connect(ui.selected_pose, &StateTable::goto_time, this, &MainGUI::set_current_time);
    connect(ui.hover_pose,    &StateTable::goto_time, this, &MainGUI::set_current_time);
    connect(ui.kf_pose,       &StateTable::goto_time, this, &MainGUI::set_current_time);

    connect(ui.tracking_step, &QPushButton::clicked, this, &MainGUI::track_single_step);

    connect(ui.show_evo_map_publisher, &QPushButton::clicked, this, &MainGUI::open_evo_map_publisher);

    // populate menu for plot button
    QMenu* plot_button_menu = new QMenu(this);
    plot_button_menu->addAction("update error plot", this, SLOT(on_plot_button_clicked()));
    plot_button_menu->addAction("plot full matrix",  this, SLOT(update_errorplot()));
    plot_button_menu->addAction("check renderer jacobian", this, SLOT(test_render_jacobian()));
    plot_button_menu->addAction("check jacobian", this, SLOT(test_general_jacobian()));
    plot_button_menu->addAction("sweep event density", this, SLOT(sweep_event_density()));
    plot_button_menu->addAction("calc mean scene depth", this, SLOT(calc_mean_scene_depth()));
    plot_button_menu->addAction("plot trajectory in color", this, SLOT(plot_trajectory()));
    plot_button_menu->addAction("SE(3) pose editor", this, SLOT(show_SE3_pose_editor()));
    ui.plot_button->setMenu(plot_button_menu);

    QMenu* minimization_button_menu = new QMenu(this);
    minimization_button_menu->addAction("minimize", this, SLOT(on_start_minimization_clicked()));
    minimization_button_menu->addAction("test gradient", this, SLOT(check_gradient()));
    ui.start_minimization->setMenu(minimization_button_menu);

    // this must occur at the end of this constructor, as they might otherwise call uninitialized things (like using world_renderer)
    //show_keyframe(data.get_frame(ros::Time(0)));

    // hide states to make some space (can be enabled in gui)
    ui.hide_hover_state->toggle();
    ui.hide_kf_state->toggle();

    YAML::Node t = config["tracking"];
    if (t["autostart"] && t["autostart"].as<bool>()) {
        // immediately start tracking
        connect(world_renderer, &WorldRenderer::ready, this, &MainGUI::start_tracking_delayed);
    }

    if (t["export_dir"]) {
        ui.opt_export_path->setText(QString::fromStdString(t["export_dir"].as<string>()));
    }

    if (t["method"]) {
        ui.opt_method->setCurrentIndex(t["method"].as<int>());
    }

    connect(&ros_spin_timer, &QTimer::timeout, this, &MainGUI::spin_ros_once);
    ros_spin_timer.start(10); // 100Hz
}

////////////////////////////////////////////////////////////////////////////////

MainGUI::~MainGUI()
{
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::closeEvent(QCloseEvent* e)
{
    QSettings settings;
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("splitter/geometry", ui.splitter->saveGeometry());
    QMainWindow::closeEvent(e);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::load_bag(const std::string& bag_path)
{
    data.load_bag(bag_path);

    string from_t = time_to_str(data.get_begin_time());
    string to_t   = time_to_str(data.get_end_time());
    ui.label_start_time->setText("Start Time: " + QString::fromStdString(from_t));
    ui.label_end_time  ->setText("End Time: "   + QString::fromStdString(to_t));
    ui.current_time->setMinimum(data.get_begin_time().toSec());
    ui.current_time->setMaximum(data.get_end_time().toSec());

    if (config["initial_pose"]) {
        set_current_time(config["initial_pose"]["stamp"].as<ros::Time>());
    }
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::set_current_time(ros::Time t)
{
    ui.current_time->setValue(t.toSec());
}

////////////////////////////////////////////////////////////////////////////////

// level = -1 -> use default (= current) images/states
std::unique_ptr<IOptimization> MainGUI::build_optimization(
        const Statef& initial_state,
        int pyramid_lvl,
        OptParams::OptCB_t live_callback)
{
    if (pyramid_lvl < 0) {
        pyramid_lvl = ui.current_pyramid_level->value()-1;
    }

    bool use_numeric_diff = false;
    if (config["tracking"]["use_numeric_diff"] and config["tracking"]["use_numeric_diff"].as<bool>()) {
        use_numeric_diff = true;
    }

    OptParams params(
            pyramid.get_eventframe(pyramid_lvl),
            pyramid.get_keyframe(pyramid_lvl),
            pyramid.get_config(pyramid_lvl),
            *world_renderer,
            (OptParams::Method) ui.opt_method->currentIndex(),
            initial_state,
            live_callback,
            use_numeric_diff);

    return IOptimization::create(params);
}

////////////////////////////////////////////////////////////////////////////////

const Keyframe& MainGUI::get_current_keyframe()
{
    return pyramid.get_keyframe(ui.current_pyramid_level->value()-1);
}

////////////////////////////////////////////////////////////////////////////////

const Eventframe& MainGUI::get_current_eventframe()
{
    return pyramid.get_eventframe(ui.current_pyramid_level->value()-1);
}

////////////////////////////////////////////////////////////////////////////////

const ImagePyramidLevelConfig& MainGUI::get_current_opt_config()
{
    return pyramid.get_config(ui.current_pyramid_level->value()-1);
}

////////////////////////////////////////////////////////////////////////////////

Statef MainGUI::get_T_WC_selected()
{
    return ui.selected_pose->getState();
}

////////////////////////////////////////////////////////////////////////////////

Statef MainGUI::get_T_WC_hover()
{
    return ui.hover_pose->getState();
}

////////////////////////////////////////////////////////////////////////////////

Statef MainGUI::get_T_WK()
{
    return ui.kf_pose->getState();
}

////////////////////////////////////////////////////////////////////////////////

// poses relative to keyframe
Posef MainGUI::get_T_KC_selected()
{
    Posef T_WK = get_current_keyframe().T_WK.pose;
    Posef T_WC = get_T_WC_selected().pose;

    return T_WK.inverse() * T_WC;
}

////////////////////////////////////////////////////////////////////////////////

Posef MainGUI::get_T_KC_hover()
{
    Posef T_WK = get_current_keyframe().T_WK.pose;
    Posef T_WC = get_T_WC_hover().pose;

    return T_WK.inverse() * T_WC;
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_time_slider_valueChanged(int)
{
    // this would all be much simpler if QAbstractSlider would support doubles as values instead of just ints :(

    QSignalBlocker blocker1(ui.current_time);
    QSignalBlocker blocker2(ui.time_slider);

    double f = ui.time_slider->value() / double(ui.time_slider->maximum() - ui.time_slider->minimum());

    double from_t = data.get_begin_time().toSec();
    double to_t   = data.get_end_time().toSec();

    ui.current_time->setValue( f * (to_t-from_t) + from_t );
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_current_time_valueChanged(double t)
{
    QSignalBlocker blocker1(ui.current_time);
    QSignalBlocker blocker2(ui.time_slider);

    double from_t = data.get_begin_time().toSec();
    double to_t   = data.get_end_time().toSec();
    double f = (t - from_t) / (to_t - from_t);

    ui.time_slider->setValue((int)( f*(ui.time_slider->maximum()-ui.time_slider->minimum()) + ui.time_slider->minimum() ));
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::show_keyframe(Keyframe kf)
{
    //kf.calc_gradient(); // 'clear/reset' gradient WHYYYY??

    pyramid.set_keyframe(kf);

    ui.kf_intensity->setPixmap(kf.intensity);

    if (kf.T_WK.valid) {
        ui.kf_pose->setState(kf.T_WK);

        cv::Mat flow_viz;
        kf.flow(flow_viz, kf.T_WK.motion, data.getCamera());
        ui.kf_flow->setPixmap(flow_viz);

        if (ui.opt_publish_ros->isChecked()) {
            rospub_keyframe_pose.publish(kf.T_WK.pose.as_msg(kf.stamp));

            // add a marker for keyframe

            visualization_msgs::Marker marker;

            marker.pose.position.x = kf.T_WK.pose.position.x();
            marker.pose.position.y = kf.T_WK.pose.position.y();
            marker.pose.position.z = kf.T_WK.pose.position.z();
            marker.pose.orientation.x = kf.T_WK.pose.orientation.x();
            marker.pose.orientation.y = kf.T_WK.pose.orientation.y();
            marker.pose.orientation.z = kf.T_WK.pose.orientation.z();
            marker.pose.orientation.w = kf.T_WK.pose.orientation.w();

            marker.scale.x = 0.075;
            marker.scale.y = 0.075;
            marker.scale.z = 0.075;

            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
            marker.color.a = 1;

            marker.lifetime = ros::Duration();

            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time(0);

            marker.ns = "keyframes"; // namespace
            marker.id = last_published_pose_marker_id++;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            rospub_current_pose_marker.publish(marker);
        }
    }

    update_keyframe_display();

    // update pose for error plot
    on_err_center_source_currentIndexChanged(-1);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_load_kf_at_t_clicked()
{
    try {
        show_keyframe(data.get_frame(ros::Time(ui.current_time->value())));
    } catch (const char* err) {
        cerr << "Failed to load KF: " << err << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_load_events_at_t_clicked()
{
    load_event_image(ros::Time(ui.current_time->value()), true);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_load_pose_at_t_clicked()
{
    ros::Time t(ui.current_time->value());
    Statef T_WC = data.get_ground_truth(t);

    if (T_WC.motion.isNull()) {
        // no velocity ground truth -> fill in velocities from IMU
        // this is the case for real-world data from the motion capture system

        T_WC.motion = data.get_imu_state(t);
        T_WC.motion.velocity = Eigen::Vector3d(0,0,0); // we only get linear *acceleration* from IMU
    }

    select_pose(T_WC);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::load_event_image(ros::Time t, bool from_middle_t)
{
    cout << "integrating events" << endl;
    Eventframe ef = data.integrate_events(t, from_middle_t);

    if (ef.img.empty()) {
        cout << "event integration failed" << endl;
        return;
    }

    cout << "integrated " << distance(ef.from_event, ef.to_event) << " events over dt = "
         << (ef.to_event->ts - ef.from_event->ts).toSec()*1000 << " ms, starting at t = " << ef.from_event->ts << endl;

    YAML::Node b = config["tracking"]["eventframe_initial_blur"];
    if (b && b.as<int>() > 0) {
        int size = b.as<int>();
        cout << "applying initial blur of " << size << endl;
        cv::GaussianBlur(ef.img, ef.img, cv::Size(size, size), 0, 0, cv::BORDER_REPLICATE);
    } else {
        cout << "no blur :(" << endl;
        cout << "node b: " << b << endl;
    }

    cv::normalize(ef.img, ef.img); // normalize so that norm == 1

    pyramid.set_eventframe(ef);
    update_eventframe_display();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::update_eventframe_display()
{
    const Eventframe& ef = get_current_eventframe();
    ui.event_frame->setPixmap(ef.img);
    on_err_center_source_currentIndexChanged(-1);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::update_keyframe_display()
{
    const Keyframe& kf = get_current_keyframe();

    if (kf.depth.empty()) {
        return;
    }

    ui.kf_depth->setPixmap(kf.depth);

    // show gradient
    ui.kf_gradient->setPixmap(kf.gradient);

    world_renderer->set_gui_camera(get_T_WK().pose);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_plot_button_clicked()
{
    if (get_current_eventframe().img.empty()) {
        load_event_image(ros::Time(ui.current_time->value()));
    }

    if (!get_current_keyframe().is_valid()) {
        cerr << "error plot failed: No keyframe" << endl;
        return;
    }


    Statef T_WC_center = get_T_WC_selected();

    if (!T_WC_center.valid) {
        cout << "invalid error plot center pose, using event frame gt" << endl;
        ui.err_center_source->setCurrentIndex(3); // 'event frame gt'
        //on_err_center_source_currentIndexChanged(3); // automatically called by currentIndexChanged signal
        T_WC_center = get_T_WC_selected();
    }

    //cout << "selected gt: " << endl << ground_truth << endl;
    //ground_truth = data.get_ground_truth( current_eventframe.from_event->ts + (current_eventframe.to_event->ts - current_eventframe.from_event->ts)*0.5 );

    cout << "center state for plotting:" << endl;
    cout << T_WC_center << endl;

    PlotRange range(
            T_WC_center,
            ui.dim1_prop->currentIndex(), ui.dim2_prop->currentIndex(),
            ui.dim1_dev->value(), ui.dim2_dev->value(),
            ui.samples_count->value());

    std::unique_ptr<IOptimization> optim = build_optimization(T_WC_center);

    ui.err_plot_lbl->plot(range, *optim);

    ui.err_plot_lbl->clearPoints();

    ui.err_plot_lbl->addPoint(PlotPoint(T_WC_center, PlotPoint::PLUS));
    ui.err_plot_lbl->addPoint(PlotPoint(data.get_ground_truth(get_current_eventframe().from_event->ts), PlotPoint::CROSS));
    ui.err_plot_lbl->addPoint(PlotPoint(data.get_ground_truth(get_current_eventframe().to_event->ts),   PlotPoint::CROSS));
}
////////////////////////////////////////////////////////////////////////////////

void MainGUI::update_errorplot()
{
    plot_matrix->show();
    plot_matrix->raise();
    plot_matrix->activateWindow();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::select_pose(const Statef& sel_pose)
{
    ui.selected_pose->setState(sel_pose);

    Posef pose = sel_pose.pose;

    if (!ui.opt_publish_ros->isChecked()) {
        return;
    }
    rospub_current_pose.publish(pose.as_msg(sel_pose.stamp));


    visualization_msgs::Marker marker;

    marker.pose.position.x = pose.position.x();
    marker.pose.position.y = pose.position.y();
    marker.pose.position.z = pose.position.z();
    marker.pose.orientation.x = pose.orientation.x();
    marker.pose.orientation.y = pose.orientation.y();
    marker.pose.orientation.z = pose.orientation.z();
    marker.pose.orientation.w = pose.orientation.w();

    /*
    marker.scale.x = 0.05; // arrow length
    marker.scale.y = 0.01; // arrow width
    marker.scale.z = 0.01; // arrow height
    */

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.02;

    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.lifetime = ros::Duration();

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time(0);

    marker.ns = "current_pose_track"; // namespace
    marker.id = last_published_pose_marker_id++;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    rospub_current_pose_marker.publish(marker);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::hover_pose(const Statef& sel_pose)
{
    ui.hover_pose->setState(sel_pose);

    if (!get_current_keyframe().is_valid()) {
        return;
    }

    const Eventframe& current_eventframe = get_current_eventframe();

    if (current_eventframe.img.empty()) {
        return;
    }

    cv::Mat projected_kf, projected_grad, projected_flow, projected_exp_change, projected_diff;
    OptVisualizationParams optviz(projected_kf, projected_grad, projected_flow, projected_exp_change, projected_diff);

    std::unique_ptr<IOptimization> optim = build_optimization(sel_pose);
    double err = optim->error(sel_pose, optviz);

    ui.correlation_value->setText(QString::number(err));

    if (ui.sel_pose_intensity_show_events->isChecked()) {
        // render events on top of projected intensity image
        data.events.plot(projected_kf, current_eventframe.from_event, current_eventframe.to_event);
    }

    cv::Mat tmp_ev_frame;
    if (ui.err_scaling_equal->isChecked()) {

        // calculate max|img| for gradient, event image and error
        double absmax = 0;
        double min_val=-1, max_val=-1;
        cv::minMaxLoc(projected_exp_change, &min_val, &max_val);
        absmax = max(abs(min_val), abs(max_val));

        cv::minMaxLoc(current_eventframe.img, &min_val, &max_val);
        absmax = max(absmax, max(abs(min_val), abs(max_val)));

        cv::minMaxLoc(projected_diff, &min_val, &max_val);
        absmax = max(absmax, max(abs(min_val), abs(max_val)));


        double err_scale = double(ui.err_scale->value()) / ui.err_scale->maximum() * 10; // slider at 0.1% -> scale by 1x

        // scale those images with the same factor, so that they're relatable
        projected_exp_change  .convertTo(projected_exp_change, CV_8UC1, 128/absmax, 128);
        current_eventframe.img.convertTo(tmp_ev_frame, CV_8UC1, 128/absmax, 128);
        projected_diff        .convertTo(projected_diff, CV_8UC1, err_scale * 128/absmax, 128);
    } else {
        // normalize images independently
        cv::normalize(projected_exp_change, projected_exp_change, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::normalize(current_eventframe.img, tmp_ev_frame, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::normalize(projected_diff, projected_diff, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }

    // render events on top of expected intensity change
    if (ui.sel_pose_exp_change_show_events->isChecked()) {
        cv::cvtColor(projected_exp_change, projected_exp_change, cv::COLOR_GRAY2BGR);
        data.events.plot(projected_exp_change, current_eventframe.from_event, current_eventframe.to_event);
    }

    // update ui
    ui.sel_pose_intensity   ->setPixmap(projected_kf, false, false);
    ui.sel_pose_gradient    ->setPixmap(projected_grad);
    ui.sel_pose_flow        ->setPixmap(projected_flow);
    ui.sel_pose_exp_change  ->setPixmap(projected_exp_change, false, false);
    ui.sel_pose_diff        ->setPixmap(projected_diff, false, false);
    ui.event_frame          ->setPixmap(tmp_ev_frame, false, false);

    world_renderer->set_gui_camera(sel_pose.pose);

    if (ui.opt_publish_ros->isChecked()) {
        // publish
        rospub_hover_pose.publish(sel_pose.pose.as_msg(sel_pose.stamp));
    }
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_err_center_source_currentIndexChanged(const int&)
{
    if (!get_current_keyframe().is_valid()) {
        cerr << "cannot set keyframe pose: no keyframe set" << endl;
        return;
    }
    const Eventframe& current_eventframe = get_current_eventframe();

    // don't trigger cellChanged when we call set_table()
    const QSignalBlocker sig_blocker(ui.selected_pose);

    Statef new_state;

    switch (ui.err_center_source->currentIndex()) {
        case 0: // 'zero'
            new_state = Statef(ros::Time(1));
            break;

        case 1: // 'kf gt'
            new_state = get_current_keyframe().T_WK;
            break;

        case 2: // 'events start gt'
            if (current_eventframe.img.empty()) {
                cerr << "cannot set event frame pose: no event frame set" << endl;
                return;
            }
            new_state = data.get_ground_truth(current_eventframe.from_event->ts);
            break;

        case 3: // 'events gt'
            if (current_eventframe.img.empty()) {
                cerr << "cannot set event frame pose: no event frame set" << endl;
                return;
            }
            new_state = data.get_ground_truth(
                    current_eventframe.from_event->ts + (current_eventframe.to_event->ts - current_eventframe.from_event->ts)*0.5);
            break;


        case 4: // 'events end gt'
            if (current_eventframe.img.empty()) {
                cerr << "cannot set event frame pose: no event frame set" << endl;
                return;
            }
            new_state = data.get_ground_truth(current_eventframe.to_event->ts);
            break;

        case 5: // 'custom'
        default:
            break; // do nothing
    }

    select_pose(new_state);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_selected_pose_cellChanged(const int&, const int&)
{
    ui.err_center_source->setCurrentIndex(5); // 'custom'
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::save_parameters(const std::string& filename)
{
    config["initial_pose"] = get_T_WC_selected();;
    config["levels"] = pyramid.toYAML();
    config["tracking"]["method"] = ui.opt_method->currentIndex();

    std::ofstream config_file(filename);
    config_file << config;
    config_file.close();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_start_minimization_clicked()
{
    if (!get_current_keyframe().is_valid()) {
        cerr << "Cannot minimize: no keyframe" << endl;
    }

    if (get_current_eventframe().img.empty()) {
        cerr << "Cannot minimize: no event image" << endl;
    }

    Statef initial_pose = get_T_WC_selected();
    if (!initial_pose.valid) {
        cerr << "Cannot minimize: invalid initial pose" << endl;
    }

    cout << "starting minimization at " << initial_pose << endl;

    std::unique_ptr<IOptimization> optim;
    if (ui.opt_show_live->isChecked()) {
        optim = build_optimization(initial_pose, -1, OptParams::OptCB_t(boost::bind(&MainGUI::solver_iter_callback, this, _1)));
    } else {
        optim = build_optimization(initial_pose);
    }
    Statef result = optim->run();

    select_pose(result);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::check_gradient()
{
    std::unique_ptr<IOptimization> optim = build_optimization(get_T_WC_selected());
    optim->check_gradient();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::solver_iter_callback(const Statef& current_state)
{
    ProfilerTimer __t__total("solve iter callback");

    hover_pose(current_state);

    if (ui.opt_draw_steps->isChecked()) {
        ui.err_plot_lbl->addPointFading(current_state);
        plot_matrix->add_point(current_state, true);

        if (jacobi_tester) {
            jacobi_tester->add_point(current_state, true);
        }
    }

    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents); // update gui
}

////////////////////////////////////////////////////////////////////////////////

bool MainGUI::track_single_step(int step_nr)
{
    ProfilerTimer __t_total("track_single_step");

    Statef s = get_T_WC_selected();
    cout << "starting tracking at " << s << endl;

    // clear points from graph
    ui.err_plot_lbl->clearPoints();
    plot_matrix->clear_points();
    if (jacobi_tester) {
        jacobi_tester->clear_points();
    }

    //ros::Duration time_step(ui.tracking_timestep_ms->value()/1000);

    double overlap = -1;
    for (int lvl = pyramid.get_size(); lvl > 0; lvl--) {

        ProfilerTimer __t_init_opt("track_single_step: building optimization");

        ui.image_pyramid->setCurrentCell(0, lvl-1);

        if (pyramid.get_config(lvl-1).rerender_mesh) {
            // calculate new keyframe
            cout << "rendering new keyframe" << endl;
            show_keyframe(world_renderer->renderPose(s.pose));
        }

        // color output differently for each level
        cout << "\033[" << (98-lvl) << "m";

        cout << "Running optimization on pyramid level " << lvl << " of " << pyramid.get_size() << endl;
        cout << "rerender? " << pyramid.get_config(lvl-1).rerender_mesh << endl;

        std::unique_ptr<IOptimization> optim;
        if (ui.opt_show_live->isChecked()) {
            optim = build_optimization(s, lvl-1, OptParams::OptCB_t(boost::bind(&MainGUI::solver_iter_callback, this, _1)));
        } else {
            optim = build_optimization(s, lvl-1);
        }

        overlap = 1;
        /*
         * TODO: implement overlap check!
        if (ui.opt_use_renderer->isChecked()) {
            overlap = 1;
        } else {
            if (lvl == 1) {
                //overlap = optim.get_overlap_fraction(s.pose);
            }
        }*/

        __t_init_opt.stop();


        s = optim->run();

        // reset color
        cout << "\033[0m";
    }



    //s.stamp += time_step;
    s.stamp = get_current_eventframe().to_event->ts; // TODO: use *MIDDLE*

    cout << "OVERLAP " << overlap*100 << "%" << endl;

    select_pose(s);
    hover_pose(s);

    if (ui.opt_export_images->isChecked()) {
        QDir base(ui.opt_export_path->text());
        base.cd("images");

        QString name = QString::number(step_nr) + ".png";

        ui.sel_pose_intensity->saveImage(base.filePath("intensity/" + name));
        ui.sel_pose_flow     ->saveImage(base.filePath("flow/" + name));
    }

    // calculate next event frame (last event frame t + tracking time step)
    //load_event_image(get_current_eventframe().from_event->ts + time_step, false);
    //
    // use consecutive integration windows of constant event size instead of a constant time step
    load_event_image(get_current_eventframe().to_event->ts, false);

    // calculate next keyframe
    cout << "rendering new keyframe" << endl;
    show_keyframe(world_renderer->renderPose(s.pose));

    return overlap < pyramid.get_config(0).overlap_fraction;
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_tracking_full_clicked()
{
    // make sure we have all the things needed to start tracking
    Statef T_WC = get_T_WC_selected();

    if (!T_WC.valid) {
        throw "Cannot start tracking: Invalid initial pose.";
    }


    if (!get_current_keyframe().is_valid()) {
        cout << "WARNING: No keyframe, generating one." << endl;
        show_keyframe(world_renderer->renderPose(T_WC.pose));
    }

    if (get_current_eventframe().img.empty()) {
        cout << "WARNING: No eventframe, generating one." << endl;
        load_event_image(T_WC.stamp);
    }


    // make sure relevant export directories exist
    QDir export_dir(ui.opt_export_path->text());

    if (export_dir.exists()) {
        if (QMessageBox::warning(this, "Export Folder Already Exists",
                    "WARNING: The export folder you've chosen already exists. Data will be overwritten! Proceed?",
                    QMessageBox::Yes | QMessageBox::Abort) != QMessageBox::Yes) {
            return;
        }
    }

    export_dir.mkpath("images/flow/");
    export_dir.mkpath("images/intensity/");


    QTextStream* csv = nullptr;
    QStringList attr_names;
    QFile* output_file = nullptr;

    if (ui.opt_export_traj->isChecked()) {
        QString filename = export_dir.filePath("track.csv");

        output_file = new QFile(filename);
        if (!output_file->open(QFile::WriteOnly)) {
            cerr << "ERROR: Failed to open file '" << filename.toStdString() << "' for writing." << endl;
            delete output_file;
            return;
        }

        csv = new QTextStream(output_file);

        *csv << "step,time";
        attr_names = Statef::get_attr_names();
        for (QString s: attr_names) {
            *csv << "," << s;
        }
        *csv << endl;
    }


    ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseStamped>("tracked_pose", 100);

    save_parameters(export_dir.filePath("config.yaml").toStdString());

    export_ground_truth_to_csv();


    QMessageBox running_msg;
    running_msg.setText("running optimization...");
    running_msg.setStandardButtons(QMessageBox::Abort);
    running_msg.show();

    ProfilerTimer __t_total_running_time("total tracking time");
    EventVector::const_iterator first_event = get_current_eventframe().from_event;

    int step_nr = 0;
    bool should_generate_new_kf;
    do {
        do {
            try {
                should_generate_new_kf = track_single_step(step_nr);
            } catch(const char* err) {
                cout << "Stopping tracking due to exception:" << endl;
                cout << err << endl;
                goto abort_tracking;
            }

            T_WC = get_T_WC_selected();

            if (csv) {
                *csv << step_nr << ",";
                *csv << QString::fromStdString(time_to_str(T_WC.stamp));
                for (size_t i = 0; i < attr_names.length(); i++) {
                    *csv << "," << T_WC.attr(i);
                }
                *csv << endl;
            }

            pose_pub.publish( T_WC.pose.as_msg(T_WC.stamp) );

            ui.current_time->setValue(T_WC.stamp.toSec());

            QCoreApplication::processEvents(); // give user the possibility to abort by clicking 'abort' on the message box
            if (running_msg.result() != 0) {
                cout << "abort requested" << endl;
                goto abort_tracking;
            }

            step_nr++;
        } while (!should_generate_new_kf);

        cout << "overlap is below " << pyramid.get_config(0).overlap_fraction*100 << "%, generating new keyframe" << endl;

        // generate new keyframe
        show_keyframe(data.get_frame(T_WC.stamp));

        // reset relative pose (but keep velocity)
        T_WC = get_T_WC_selected();
        T_WC.pose.position = Eigen::Translation3d(0,0,0);
        T_WC.pose.orientation = Eigen::Quaterniond(1,0,0,0);
        select_pose(T_WC);

    } while(T_WC.stamp < data.get_end_time());
abort_tracking:

    if (csv) {
        delete csv;
    }
    if (output_file) {
        output_file->close();
        delete output_file;
    }

    running_msg.close();


    nanoseconds total_t = __t_total_running_time.stop();
    EventVector::const_iterator last_event = get_current_eventframe().to_event;
    long int total_event_count = distance(first_event, last_event);

    double comp_time = total_t.count()/1000.0/1000.0/1000.0;
    double real_time = (last_event->ts - first_event->ts).toSec();


    cout << "-------------------------------------------" << endl;
    cout << "Tracking Timings:" << endl;
    cout << "Total computation time: " << comp_time << " sec" << endl;
    cout << "Total real time: " << real_time << " sec" << endl;
    cout << "Giving us a slowdown of: " << comp_time/real_time << "x" << endl;
    cout << "Number of tracking steps: " << step_nr << " = " << total_t.count()/1000.0/1000.0/step_nr << " ms/step" << endl;
    cout << "Number of events processed: " << total_event_count << " = " << double(total_event_count)/duration_cast<milliseconds>(total_t).count()*1000.0 << " events/sec" << endl;
    cout << "-------------------------------------------" << endl;
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_current_pyramid_level_valueChanged(int v)
{
    ui.current_pyramid_level_lbl->setText(QString::number(v));
    update_keyframe_display();
    update_eventframe_display();

    hover_pose(get_T_WC_selected());
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_publish_gt_clicked()
{
    data.publish_gt_track(rospub_gt_track);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_publish_map_clicked()
{
    world_renderer->publish(rospub_map);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_pyramid_add_clicked()
{
    pyramid.set_size(pyramid.get_size()+1);
    ui.current_pyramid_level->setMaximum(pyramid.get_size());
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_pyramid_remove_clicked()
{
    pyramid.set_size(pyramid.get_size()-1);
    ui.current_pyramid_level->setMaximum(pyramid.get_size());
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_generate_keyframe_clicked()
{
    Posef T_WC = get_T_WC_selected().pose;
    show_keyframe(world_renderer->renderPose(T_WC));
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_opt_choose_export_path_clicked()
{
    QString filename = QFileDialog::getExistingDirectory(this,
            tr("Export folder for tracking data"),
            ui.opt_export_path->text(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (filename.isEmpty()) {
        return;
    }

    ui.opt_export_path->setText(filename);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::start_tracking_delayed()
{
    // only start tracking when main loop is running
    // otherwise GUI won't be visible
    QMetaObject::invokeMethod(this, SLOT(on_tracking_full_clicked()), Qt::QueuedConnection);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::export_ground_truth_to_csv()
{
    QDir export_dir(ui.opt_export_path->text());

    QString filename = export_dir.filePath("ground_truth.csv");

    QFile out_file(filename);
    if (!out_file.open(QFile::WriteOnly)) {
        cerr << "ERROR: Failed to open file '" << filename.toStdString() << "' for writing." << endl;
        return;
    }

    QTextStream csv(&out_file);

    // write header
    csv << "time";
    for (QString s: Statef::get_attr_names()) {
        csv << "," << s;
    }
    csv << endl;

    // write ground truth poses
    for (Statef s: data.get_ground_truth_vect()) {
        csv << QString::fromStdString(time_to_str(s.stamp));
        for (size_t i = 0; i < Statef::get_attr_names().length(); i++) {
            csv << "," << s.attr(i);
        }
        csv << endl;
    }

    out_file.close();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_event_density_valueChanged(double d)
{
    QSignalBlocker blocker(ui.event_count);

    data.event_count_density = d;
    config["tracking"]["event_count_density"] = d;
    ui.event_count->setValue(d * data.getCamera().getCameraWidth() * data.getCamera().getCameraHeight());
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_event_count_valueChanged(int c)
{
    QSignalBlocker blocker(ui.event_density);

    double d = c / (data.getCamera().getCameraWidth() * data.getCamera().getCameraHeight());
    data.event_count_density = d;
    config["tracking"]["event_count_density"] = d;
    ui.event_density->setValue(d);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::test_render_jacobian()
{
    RenderJacobiTester* tester = new RenderJacobiTester(get_T_WC_selected().pose, *world_renderer, data.getCamera(), ui, this);
    tester->show();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::test_general_jacobian()
{
    // no need to delete previous panel, Qt will clean it up

    jacobi_tester = new GeneralJacobiTester(
            build_optimization(get_T_WC_selected()),
            ui, this);
    jacobi_tester->show();

    connect(jacobi_tester, &GeneralJacobiTester::stateHovered, this, &MainGUI::hover_pose);
    connect(jacobi_tester, &GeneralJacobiTester::stateSelected, this, &MainGUI::select_pose);
};

////////////////////////////////////////////////////////////////////////////////

void MainGUI::on_save_params_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
            tr("Export folder for tracking data"),
            ui.opt_export_path->text(),
            "config file (*.yaml)"
    );

    if (filename.isEmpty()) {
        return;
    }

    config["tracking"]["export_dir"] = ui.opt_export_path->text().toStdString();

    save_parameters(filename.toStdString());
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::adjust_current_pose(Eigen::Isometry3d t)
{
    Statef T = get_T_WC_selected();
    T.pose = T.pose * Posef(t);
    select_pose(T);
    show_keyframe(world_renderer->renderPose(T.pose));
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::sweep_event_density()
{
    const std::vector<double> densities({
            0.01,
            0.03,
            0.05,
            0.1,
            0.15,
            0.2,
            0.25,
            0.3,
            0.4,
            0.5,
            });

    // choose folder to save images to
    QString filename = QFileDialog::getExistingDirectory(this,
            tr("Choose folder to save images to"),
            ui.opt_export_path->text(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
    );

    if (filename.isEmpty()) {
        return;
    }

    QDir export_dir(filename);

    for (double density: densities) {
        cout << "Sweeping event density. Rendering density of " << density << " events / pixel" << endl;

        // set event density
        ui.event_density->setValue(density);

        // load and integrate events
        on_load_events_at_t_clicked();

        // set center for plot to middle of event integration window
        const Eventframe& current_eventframe = get_current_eventframe();
        Statef new_state = data.get_ground_truth(
                current_eventframe.from_event->ts + (current_eventframe.to_event->ts - current_eventframe.from_event->ts)*0.5);
        select_pose(new_state);
        hover_pose(new_state);

        // save image with events on top
        ui.sel_pose_intensity->saveImage(export_dir.filePath("intensity_"  + QString::number(density) + ".png"));
        ui.event_frame       ->saveImage(export_dir.filePath("eventframe_" + QString::number(density) + ".png"));

        // render plot
        on_plot_button_clicked();

        // save plot
        ui.err_plot_lbl->saveGUIimage(export_dir.filePath("err_gui_" + QString::number(density) + ".png"));
        ui.err_plot_lbl->saveImage   (export_dir.filePath("err_" + QString::number(density) + ".png"));
    }
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::calc_mean_scene_depth()
{
    std::vector<double> scene_depths;
    ros::Time t = data.get_begin_time();
    while (t <= data.get_end_time()) {
        cout << "rendering pose at t = " << t << endl;

        Statef s = data.get_ground_truth(t);
        Keyframe k = world_renderer->renderPose(s.pose);

        cv::Scalar c = cv::mean(k.depth, k.depth > 0);
        cout << " --> mean = " << c[0] << endl;
        scene_depths.push_back(c[0]);

        t += ros::Duration(0.1); // increment by 100ms
    }

    std::nth_element(scene_depths.begin(), scene_depths.begin() + scene_depths.size()/2, scene_depths.end());

    cout << "TOTAL MEAN SCENE DEPTH: " << scene_depths[scene_depths.size()/2] << endl;
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::plot_trajectory()
{
    QString filename = QFileDialog::getOpenFileName(this,
            tr("Poses to render"),
            ui.opt_export_path->text(),
            "trajectory file (*.csv)"
    );

    if (filename.isEmpty()) {
        return;
    }

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        cerr << "ERROR: failed to open file" << endl;
    }



    QDir export_dir(ui.opt_export_path->text());
    export_dir.mkpath("images/color/");
    export_dir.mkpath("images/color_with_events/");
    export_dir.mkpath("images/events_only/");
    export_dir.mkpath("images/depth/");

    world_renderer->setupHiresRendering(4);

    QTextStream csv(&file);
    bool firstline = true;
    while (!csv.atEnd()) {
        QString line = csv.readLine();

        // skip header
        if (firstline) {
            firstline = false;
            continue;
        }


        const QStringList row = line.split(',');

        if (row.length() < 15) {
            cout << "WARNING: found invalid row, aborting" << endl;
            break;
        }

        Statef s(row);

        cv::Mat col, depth;
        world_renderer->renderPoseHighres(s.pose, col, depth);

        col.convertTo(col, CV_8UC3, 255.0f);

        imwrite(export_dir.filePath("images/color/" + row[0] + ".png").toStdString(), col);
        imwrite(export_dir.filePath("images/depth/" + row[0] + ".png").toStdString(), depth);


        Eventframe ef = data.integrate_events(s.stamp, true);

        if (ef.img.empty()) {
            cout << "event integration failed" << endl;
            return;
        }

        // draw events onto color image
        data.events.plot(col, ef.from_event, ef.to_event);
        imwrite(export_dir.filePath("images/color_with_events/" + row[0] + ".png").toStdString(), col);

        // draw events on empty image
        col = cv::Scalar(255,255,255);
        data.events.plot(col, ef.from_event, ef.to_event);
        imwrite(export_dir.filePath("images/events_only/" + row[0] + ".png").toStdString(), col);

        cout << "rendered idx " << row[0].toStdString() << endl;

        //if (row[0].toInt() > 5) break;
    }

    world_renderer->cleanupHiresRendering();
    cout << "done" << endl;
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::show_SE3_pose_editor()
{
    // no need to delete previous panel, Qt will clean it up

    auto dialog = new SE3PoseEditor(this);
    dialog->setReferencePose(get_T_WC_selected());
    dialog->show();

    connect(dialog, &SE3PoseEditor::stateSelected, this, &MainGUI::hover_pose);
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::open_evo_map_publisher()
{
    if (!evo_map_publisher) {
        evo_map_publisher = new EvoMapPublisher(node, world_renderer, this);
    }

    evo_map_publisher->show();
}

////////////////////////////////////////////////////////////////////////////////

void MainGUI::spin_ros_once()
{
    ros::spinOnce();
}

////////////////////////////////////////////////////////////////////////////////

