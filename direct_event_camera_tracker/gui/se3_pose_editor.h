#ifndef SE3_POSE_EDITOR_H_GFSAVJFA
#define SE3_POSE_EDITOR_H_GFSAVJFA

#include <Eigen/Geometry>
#include <QDialog>
#include <QTableWidget>
#include <QTableWidgetItem>

#include "point.h"

class SE3PoseEditor : public QDialog
{
    typedef Eigen::Matrix<double,6,1> Vec6;

    Q_OBJECT
public:
    SE3PoseEditor(
            QWidget* parent = 0);

    QStringList get_attr_names();

    Vec6 getTwist();
    Posef getPose();

public slots:
    void setReferencePose(const Statef& s);
    void publishState();

signals:
    void stateSelected(const Statef& s);

private:
    QTableWidget* table;
    Posef T_world_ref;
};

#endif /* end of include guard: SE3_POSE_EDITOR_H_GFSAVJFA */
