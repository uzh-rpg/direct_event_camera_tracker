#include "se3_pose_editor.h"

#include <QBoxLayout>
#include <QPushButton>

using namespace std;

////////////////////////////////////////////////////////////////////////////////

SE3PoseEditor::SE3PoseEditor(
            QWidget* parent)
    : QDialog(parent)
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    table = new QTableWidget(this);
    layout->addWidget(table);

    table->setRowCount(6);
    table->setColumnCount(1);

    table->setVerticalHeaderLabels(get_attr_names());

    for (size_t i = 0; i < 6; ++i) {
        table->setItem(i, 0, new QTableWidgetItem("0"));
    }

    connect(table, &QTableWidget::itemChanged, this, &SE3PoseEditor::publishState);

    setLayout(layout);
}

////////////////////////////////////////////////////////////////////////////////

QStringList SE3PoseEditor::get_attr_names()
{
    QStringList attrs;
    attrs
        << "Position X"
        << "Position Y"
        << "Position Z"
        << "Orientation X"
        << "Orientation Y"
        << "Orientation Z";
    return attrs;
}

////////////////////////////////////////////////////////////////////////////////

SE3PoseEditor::Vec6 SE3PoseEditor::getTwist()
{
    Vec6 v; v <<
            table->item(0, 0)->text().toDouble(),
            table->item(1, 0)->text().toDouble(),
            table->item(2, 0)->text().toDouble(),
            table->item(3, 0)->text().toDouble(),
            table->item(4, 0)->text().toDouble(),
            table->item(5, 0)->text().toDouble();
    return v;
}

////////////////////////////////////////////////////////////////////////////////

Posef SE3PoseEditor::getPose()
{
    using Scalar = double;

    Eigen::Transform<Scalar, 3, Eigen::Isometry> T_WC = T_world_ref.T().cast<Scalar>();
    T_WC *= Sophus::SE3<Scalar>::exp(getTwist()).matrix();
    return Posef(T_WC);
}

////////////////////////////////////////////////////////////////////////////////

void SE3PoseEditor::setReferencePose(const Statef& s)
{
    T_world_ref = s.pose;
}

////////////////////////////////////////////////////////////////////////////////

void SE3PoseEditor::publishState()
{
    emit stateSelected(Statef(ros::Time(0), getPose()));
}

////////////////////////////////////////////////////////////////////////////////
