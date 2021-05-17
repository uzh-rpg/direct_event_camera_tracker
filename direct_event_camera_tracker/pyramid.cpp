#include "pyramid.h"

using namespace std;


////////////////////////////////////////////////////////////////////////////////

#define ImagePyramidLevelConfig_ATTRS \
    X(resample, unsigned int) \
    X(blur, unsigned int) \
    X(rerender_mesh, bool) \
    X(max_iterations, unsigned int) \
    X(overlap_fraction, float) \
    X(subset_fraction, float) \
    X(fix_velocity, bool) \
    X(solver_function_tolerance, double) \
    X(solver_gradient_tolerance, double) \
    X(solver_parameter_tolerance, double) \
    X(loss_function_scaling, double)

////////////////////////////////////////////////////////////////////////////////

ImagePyramidLevelConfig::ImagePyramidLevelConfig(boost::optional<std::vector<ImagePyramidLevelConfig>::iterator> parent)
  : resample(1),
    blur(0),
    max_iterations(50),
    overlap_fraction(0.6),
    subset_fraction(1),
    fix_velocity(false),

    // defaults of CERES:
    solver_function_tolerance(1e-6),
    solver_gradient_tolerance(1e-10),
    solver_parameter_tolerance(1e-8),

    loss_function_scaling(-1),

    rerender_mesh(false),

    parent(parent)
{
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramidLevelConfig::set_from_yaml(YAML::Node node)
{

#define X(key, type) do { \
    if (node[#key]) { \
        key = node[#key].as<type>(); \
        /* yaml-cpp 0.5.2 (default on Ubuntu 16.04) is broken :( */ \
        /* see https://github.com/jbeder/yaml-cpp/pull/303 */ \
        /* node.remove( std::string(#key) ); */ \
    } \
} while (0);

    ImagePyramidLevelConfig_ATTRS

#undef X

    // only allow odd values (otherwise OpenCV complains)
    if (blur > 0 && blur % 2 == 0) {
        blur++;
    }

    /*
     * we can't check this, as YAML::Node::remove() is broken (see above)
    if (node.size() > 0) {
        cerr << "WARNING: Unhandled parameter[s] in pyramid level config:" << endl;
        cerr << node << endl;
    }
    */
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramidLevelConfig::init_table(QTableWidget& table)
{
    QStringList headers;

#define X(key, type) headers << #key;

    ImagePyramidLevelConfig_ATTRS

#undef X

    table.setRowCount(headers.size());
    table.setVerticalHeaderLabels(headers);
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramidLevelConfig::set_from_table_column(const QTableWidget& table, const int col)
{
    unsigned int row = 0;

#define X(key, type) do { \
    key = qvariant_cast<type>(table.item(row,col)->text()); \
    row++; \
} while (0);

    ImagePyramidLevelConfig_ATTRS

#undef X
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramidLevelConfig::write_to_table_column(QTableWidget& table, const int col) const
{
    // make sure row exists
    if (table.columnCount() <= col) {
        table.setColumnCount(col+1);
    }

    unsigned int row = 0;

#define X(key, type) do { \
    /* make sure cell contains a QTableWidgetItem */ \
    if (!table.item(row, col)) { \
        table.setItem(row, col, new QTableWidgetItem("n/a")); \
    } \
    table.item(row,col)->setText(QString::number(key)); \
    row++; \
} while (0);

    ImagePyramidLevelConfig_ATTRS

#undef X
}

////////////////////////////////////////////////////////////////////////////////

namespace YAML {
Node convert<ImagePyramidLevelConfig>::encode(const ImagePyramidLevelConfig& rhs)
{
    YAML::Node node;

#define X(key, type) do { \
    node[#key] = rhs.key; \
} while (0);

    ImagePyramidLevelConfig_ATTRS

#undef X

    return node;
}

////////////////////////////////////////////////////////////////////////////////

bool convert<ImagePyramidLevelConfig>::decode(const Node& node, ImagePyramidLevelConfig& lvl)
{
#define X(key, type) do { \
    if (node[#key]) { \
        lvl.key = node[#key].as<type>(); \
        /* yaml-cpp 0.5.2 (default on Ubuntu 16.04) is broken :( */ \
        /* see https://github.com/jbeder/yaml-cpp/pull/303 */ \
        /* node.remove( std::string(#key) ); */ \
    } \
} while (0);

    ImagePyramidLevelConfig_ATTRS

#undef X

    // only allow odd values (otherwise OpenCV complains)
    if (lvl.blur > 0 && lvl.blur % 2 == 0) {
        lvl.blur++;
    }

    /*
     * we can't check this, as YAML::Node::remove() is broken (see above)
    if (node.size() > 0) {
        cerr << "WARNING: Unhandled parameter[s] in pyramid level config:" << endl;
        cerr << node << endl;
        return false;
    }
    */

    return true;
}
}

////////////////////////////////////////////////////////////////////////////////

double ImagePyramidLevelConfig::getTotalBlur() const
{
    if (parent) {
        // multiple gaussian blurs are the same as applying a single one with size sqrt(sum(r_i^2))

        double b1 = (*parent)->getTotalBlur();
        double b2 = this->blur;

        return sqrt( b1*b1 + b2*b2 );
    } else {
        return this->blur;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ImagePyramid::ImagePyramid(const YAML::Node& config)
    : linked_table(nullptr)
{
    //cout << "loading img pyramid config:" << endl;
    //cout << config << endl;

    if (!config["levels"]) {
        throw "Invalid image pyramid config file: no level defined.";
    }

    size = config["levels"].size();

    ImagePyramidLevelConfig default_cfg;

    if (config["pyramid_defaults"]) {
        default_cfg.set_from_yaml(config["pyramid_defaults"]);
    }

    for (const YAML::Node& n: config["levels"]) {
        ImagePyramidLevelConfig lvl = default_cfg;
        lvl.set_from_yaml(n);
        configs.push_back(lvl);
        if (configs.size() >= 2) {
            (configs.end()-1)->parent = configs.end()-2;
        }
    }

    keyframes  .resize(size);
    eventframes.resize(size);
}

////////////////////////////////////////////////////////////////////////////////

/*
ImagePyramid::ImagePyramid(const size_t size)
    : size(size), keyframes(size), eventframes(size)
{
}

////////////////////////////////////////////////////////////////////////////////

ImagePyramid::ImagePyramid(const size_t size, const size_t initial_blur, const Keyframe& kf, const Eventframe& ef)
    : size(size), keyframes(size), eventframes(size)
{
    Keyframe blurred_kf = kf;
    cv::GaussianBlur(kf.gradient, blurred_kf.gradient, cv::Size(initial_blur, initial_blur), 0, 0, cv::BORDER_REPLICATE);

    Eventframe blurred_ef = ef;
    cv::GaussianBlur(ef.img, blurred_ef.img, cv::Size(initial_blur, initial_blur), 0, 0, cv::BORDER_REPLICATE);

    set_keyframe  (blurred_kf);
    set_eventframe(blurred_ef);
}
*/

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::set_keyframe(const Keyframe& kf)
{
    if (&keyframes[0] != &kf) {
        keyframes[0] = kf;
        update_keyframes();
    } else {
        cout << "Pyramid: not updating keyframe, is already set." << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::update_keyframes(const size_t from_lvl)
{
    for (size_t i = from_lvl; i < size; i++) {

        // copy image from lower level (if any)
        // TODO: no need to do a full copy if we're going to downsample it immediately afterwards
        // -> but: copy it if we're blurring it!
        if (i > 0) {
            keyframes[i] = keyframes[i-1].copy();
        }

        // apply config to this level

        // downsampling
        if (configs[i].resample > 0) {
            for (size_t k = 0; k < configs[i].resample; k++) {
                keyframes[i] = keyframes[i].downsample2();
            }
        }

        // blur
        keyframes[i].blur(configs[i].blur);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::set_eventframe(const Eventframe& ef)
{
    eventframes[0] = ef;
    update_eventframes();
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::update_eventframes(const size_t from_lvl)
{
    for (size_t i = from_lvl; i < size; i++) {

        // copy image from lower level (if any)
        if (i > 0) {
            eventframes[i-1].img.copyTo(eventframes[i].img);
            eventframes[i].from_event = eventframes[i-1].from_event;
            eventframes[i].to_event   = eventframes[i-1].to_event;
        }

        // apply config to this level

        // downsampling
        if (configs[i].resample > 0) {
            for (size_t k = 0; k < configs[i].resample; k++) {
                downsample2_img(eventframes[i].img, eventframes[i].img);
            }
        }

        // blur
        if (configs[i].blur > 0) {
            cv::GaussianBlur(eventframes[i].img, eventframes[i].img, cv::Size(configs[i].blur, configs[i].blur), 0, 0, cv::BORDER_REPLICATE);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::set_size(const size_t new_size)
{
    size_t old_size = size;
    size = new_size;

    keyframes  .resize(new_size);
    eventframes.resize(new_size);
    configs    .resize(new_size);

    if (new_size > old_size) {
        // calculate additional pyramid levels
        update_keyframes  (old_size-1);
        update_eventframes(old_size-1);
    }

    if (linked_table) {

        // remove all columns
        while (linked_table->columnCount() > 0) {
            linked_table->removeColumn(0);
        }

        // re-set them
        link_table(*linked_table);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::load_settings_from_table(const QTableWidget& table)
{
    for (int column = 0; column < size; column++) {
        configs[column].set_from_table_column(table, column);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::write_settings_to_table(QTableWidget& table) const
{
    if (table.columnCount() == 0) {
        ImagePyramidLevelConfig::init_table(table);
    }

    for (int column = 0; column < size; column++) {
        configs[column].write_to_table_column(table, column);
    }
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::link_table(QTableWidget& table)
{
    if (linked_table) {
        disconnect(linked_table, &QTableWidget::itemChanged, this, &ImagePyramid::on_linked_table_changed);
    }

    write_settings_to_table(table);

    connect(&table, &QTableWidget::itemChanged, this, &ImagePyramid::on_linked_table_changed);

    linked_table = &table;
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::on_linked_table_changed(QTableWidgetItem* item)
{
    assert(item->column() < size);
    assert(item->tableWidget() == linked_table);

    configs[item->column()].set_from_table_column(*item->tableWidget(), item->column());
}

////////////////////////////////////////////////////////////////////////////////

YAML::Node ImagePyramid::toYAML()
{
    YAML::Node lvls;

    size_t i = 0;
    for (const ImagePyramidLevelConfig& cfg: configs) {
        lvls[i++] = cfg;
    }

    return lvls;
}

////////////////////////////////////////////////////////////////////////////////

void ImagePyramid::checkLvl(size_t level) const
{
    if (level >= get_size()) {
        //throw std::exception((QString("Invalid pyramid level requested: ") + level + ", but pyramid only has " + get_size() + " levels.").constData());
        throw "Invalid pyramid level requested.";
    }
}

////////////////////////////////////////////////////////////////////////////////

