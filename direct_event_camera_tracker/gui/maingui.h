#ifndef MAINGUI_H_ZFRDYPXZ
#define MAINGUI_H_ZFRDYPXZ

#include <memory>
#include <algorithm>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/optional/optional_io.hpp>
#include <QMainWindow>
#include <QSettings>
#include <QSpinBox>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QDir>
#include <QMetaObject>
#include <QPushButton>
#include <QTimer>

#include "ui_maingui.h"

#include "datasource.h"
#include "optimization/optimization.h"
#include "optimization/opt_forward_warp.h"
#include "optimization/opt_rerender.h"
#include "gui/plotmatrix.h"
#include "gui/render_jacobi_tester.h"
#include "gui/general_jacobi_tester.h"
#include "gui/evo_map_publisher.h"
#include "gui/se3_pose_editor.h"
#include "pyramid.h"
#include "opengl/world_renderer.h"
#include "point_yaml.h"
#include "utils/profiler.h"

class MainGUI : public QMainWindow
{
    Q_OBJECT

public:
    MainGUI(const YAML::Node& config);
    ~MainGUI();

    void closeEvent(QCloseEvent* e);

    void load_bag(const std::string& bag_path);

    void show_keyframe(Keyframe kf);

    void solver_iter_callback(const Statef& current_state);

    const Keyframe&   get_current_keyframe();
    const Eventframe& get_current_eventframe();
    const ImagePyramidLevelConfig& get_current_opt_config();

    // absolute poses & motions
    Statef get_T_WC_selected();
    Statef get_T_WC_hover();
    Statef get_T_WK();

    // poses relative to keyframe
    Posef get_T_KC_selected();
    Posef get_T_KC_hover();

    // TODO: can we set a default value for initial_state?
    std::unique_ptr<IOptimization> build_optimization(
        const Statef& initial_state,
        int pyramid_lvl = -1,
        OptParams::OptCB_t live_callback = {}
    );

    const Ui::MainWindow& getUI() { return ui; }

public slots:
    void set_current_time(ros::Time t);
    void save_parameters(const std::string& filename);

private slots:

    void on_plot_button_clicked();
    void update_errorplot();

    void select_pose(const Statef& p);
    void hover_pose(const Statef& p);

    void on_err_center_source_currentIndexChanged(const int&);
    void on_selected_pose_cellChanged(const int& row, const int& col);

    void on_start_minimization_clicked();
    void check_gradient();

    void on_time_slider_valueChanged(int v);
    void on_current_time_valueChanged(double t);

    void on_load_kf_at_t_clicked();
    void on_load_events_at_t_clicked();
    void on_load_pose_at_t_clicked();

    void load_event_image(ros::Time t, bool from_middle_t=false);

    bool track_single_step(int step_nr = 0);
    void on_tracking_full_clicked();

    void on_current_pyramid_level_valueChanged(int v);

    void update_eventframe_display();
    void update_keyframe_display();

    void on_publish_gt_clicked();
    void on_publish_map_clicked();

    void on_pyramid_add_clicked();
    void on_pyramid_remove_clicked();

    void on_generate_keyframe_clicked();

    void on_opt_choose_export_path_clicked();

    void start_tracking_delayed();

    void export_ground_truth_to_csv();

    void on_event_density_valueChanged(double d);
    void on_event_count_valueChanged(int c);

    void test_render_jacobian();
    void test_general_jacobian();

    void on_save_params_clicked();

    void adjust_current_pose(Eigen::Isometry3d t);
    void adjust_current_pose(Eigen::AngleAxisd r)    { adjust_current_pose(Eigen::Isometry3d(r)); }
    void adjust_current_pose(Eigen::Translation3d t) { adjust_current_pose(Eigen::Isometry3d(t)); }

    void on_pose_transl_x_min_clicked()  { adjust_current_pose(Eigen::Translation3d(-ui.dim1_dev->value(), 0, 0)); }
    void on_pose_transl_x_plus_clicked() { adjust_current_pose(Eigen::Translation3d( ui.dim1_dev->value(), 0, 0)); }
    void on_pose_transl_y_min_clicked()  { adjust_current_pose(Eigen::Translation3d(0, -ui.dim1_dev->value(), 0)); }
    void on_pose_transl_y_plus_clicked() { adjust_current_pose(Eigen::Translation3d(0,  ui.dim1_dev->value(), 0)); }
    void on_pose_transl_z_min_clicked()  { adjust_current_pose(Eigen::Translation3d(0, 0, -ui.dim1_dev->value())); }
    void on_pose_transl_z_plus_clicked() { adjust_current_pose(Eigen::Translation3d(0, 0,  ui.dim1_dev->value())); }

    void on_pose_rot_x_min_clicked()  { adjust_current_pose(Eigen::AngleAxisd(-ui.dim2_dev->value(), Eigen::Vector3d::UnitX())); }
    void on_pose_rot_x_plus_clicked() { adjust_current_pose(Eigen::AngleAxisd( ui.dim2_dev->value(), Eigen::Vector3d::UnitX())); }
    void on_pose_rot_y_min_clicked()  { adjust_current_pose(Eigen::AngleAxisd(-ui.dim2_dev->value(), Eigen::Vector3d::UnitY())); }
    void on_pose_rot_y_plus_clicked() { adjust_current_pose(Eigen::AngleAxisd( ui.dim2_dev->value(), Eigen::Vector3d::UnitY())); }
    void on_pose_rot_z_min_clicked()  { adjust_current_pose(Eigen::AngleAxisd(-ui.dim2_dev->value(), Eigen::Vector3d::UnitZ())); }
    void on_pose_rot_z_plus_clicked() { adjust_current_pose(Eigen::AngleAxisd( ui.dim2_dev->value(), Eigen::Vector3d::UnitZ())); }

    void sweep_event_density();
    void calc_mean_scene_depth();
    void plot_trajectory();
    void show_SE3_pose_editor();

    void open_evo_map_publisher();

    void spin_ros_once();

private:
    Ui::MainWindow ui;
    YAML::Node config;

    WorldRenderer* world_renderer;
    GeneralJacobiTester* jacobi_tester;
    EvoMapPublisher* evo_map_publisher;

    ros::NodeHandle node;
    Datasource data;
    QTimer ros_spin_timer;

    ImagePyramid pyramid;

    PlotMatrix* plot_matrix;

    ros::Publisher rospub_hover_pose;
    ros::Publisher rospub_current_pose;
    ros::Publisher rospub_keyframe_pose;
    ros::Publisher rospub_gt_track;
    ros::Publisher rospub_current_pose_marker;
    int last_published_pose_marker_id;
    ros::Publisher rospub_map;
};

#endif /* end of include guard: MAINGUI_H_ZFRDYPXZ */
