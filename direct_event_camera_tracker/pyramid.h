#ifndef PYRAMID_H_FWBE3KSV
#define PYRAMID_H_FWBE3KSV

// has to be included before X11.h due to https://github.com/jbeder/yaml-cpp/issues/320
#include <yaml-cpp/yaml.h>

#include <tuple>
#include <vector>
#include <boost/optional.hpp>

#include "keyframe.h"
#include "event_buffer.h"

////////////////////////////////////////////////////////////////////////////////

struct ImagePyramidLevelConfig
{
    ImagePyramidLevelConfig(boost::optional<std::vector<ImagePyramidLevelConfig>::iterator> parent = {});

    void set_from_yaml(YAML::Node node);

    static void init_table(QTableWidget& table);
    void set_from_table_column(const QTableWidget& table, const int column);
    void write_to_table_column(QTableWidget& table, const int column) const;

    unsigned int resample; // number of times to halve image size

    // this value is relative to the previous iteration for keyframe-based optmization
    unsigned int blur; // amount of blur to apply (= size of kernel; 0 to disable; must be odd; relative to previous level)
    // but absolute when using OpenGL to render a frame for every perspective:
    double getTotalBlur() const;

    unsigned int max_iterations; // max. iteration count for optimization
    float overlap_fraction; // min. percentage of pixels that must overlap -> trigger new keyframe otherwise
    double subset_fraction; // [0 to 1] how many points should be used (based on strength of their gradients)

    bool fix_velocity; // only optimize pose

    // solver termination criteria
    double solver_function_tolerance;
    double solver_gradient_tolerance;
    double solver_parameter_tolerance;

    double loss_function_scaling; // <0 = disable loss function

    bool rerender_mesh; // trigger creation of new keyframe (doesn't affect rerenderer)

    boost::optional<std::vector<ImagePyramidLevelConfig>::iterator> parent;
};

////////////////////////////////////////////////////////////////////////////////

namespace YAML
{
    template<>
    struct convert<ImagePyramidLevelConfig> {
        static Node encode(const ImagePyramidLevelConfig& rhs);
        static bool decode(const Node& node, ImagePyramidLevelConfig& rhs);
    };
}

////////////////////////////////////////////////////////////////////////////////

class ImagePyramid : public QObject
{
    Q_OBJECT

public:
    ImagePyramid(const YAML::Node& config);
    //ImagePyramid(const size_t size);
    //ImagePyramid(const size_t size, const size_t initial_blur, const Keyframe& kf, const Eventframe& ef);

    size_t get_size() const { return size; }

    const Keyframe&   get_keyframe  (size_t level) const { return keyframes  [level]; }
    const Eventframe& get_eventframe(size_t level) const { return eventframes[level]; }
    const ImagePyramidLevelConfig& get_config(size_t level) const { return configs[level]; }

    void set_keyframe  (const Keyframe& kf);
    void set_eventframe(const Eventframe& ef);

    void update_keyframes  (const size_t from_lvl=0);
    void update_eventframes(const size_t from_lvl=0);

    void set_size(const size_t new_size);
    void link_table(QTableWidget& table);
    void load_settings_from_table(const QTableWidget& table);
    void write_settings_to_table(QTableWidget& table) const;

    YAML::Node toYAML();

    void checkLvl(size_t level) const;

protected slots:
    void on_linked_table_changed(QTableWidgetItem* item);

protected:
    QTableWidget* linked_table;

    size_t size;
    std::vector<Keyframe> keyframes;
    std::vector<Eventframe> eventframes;
    std::vector<ImagePyramidLevelConfig> configs;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: PYRAMID_H_FWBE3KSV */
