#include "point.h"

using namespace Eigen;
using namespace std;

/*
Pose Pose::operator-(const Pose& T_WA) const
{
    return Pose( T_WA.T().inverse() * this->T() );
}
*/


////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
void State<Scalar>::init_table(QTableWidget& table)
{
    if (table.rowCount() != 0) {
        assert(table.rowCount() == 14);
        return;
    }

    table.setRowCount(14);
    table.setColumnCount(1);

    QStringList headers;
    headers << "Time";
    headers += get_attr_names();

    table.setVerticalHeaderLabels(headers);

    for (size_t i = 0; i < 14; i++) { // 12 DoF + timestamp + Quaternion W
        table.setItem(i, 0, new QTableWidgetItem("n/a"));
    }
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
void State<Scalar>::set_table(const QTableWidget& table) const
{
    if (!valid) {
        return;
    }

    // timestamp
    table.item(0, 0)->setText(QString::fromStdString(time_to_str(stamp)));

    // attributes
    for (size_t i = 0; i < 13; i++) {
        table.item(i+1, 0)->setText(QString::number(attr(i)));
    }
}

////////////////////////////////////////////////////////////////////////////////

template <typename Scalar>
State<Scalar> State<Scalar>::from_table(const QTableWidget& table)
{
    assert(table.rowCount() == 14);
    assert(table.columnCount() == 1);

    return State<Scalar>(
            ros::Time(table.item(0, 0)->text().toDouble()),
            Pose<Scalar>(
                Eigen::Translation3d(
                    table.item(1, 0)->text().toDouble(),  // X
                    table.item(2, 0)->text().toDouble(),  // Y
                    table.item(3, 0)->text().toDouble()), // Z
                Eigen::Quaterniond(table.item(7, 0)->text().toDouble(), // W
                    table.item(4, 0)->text().toDouble(),  // X
                    table.item(5, 0)->text().toDouble(),  // Y
                    table.item(6, 0)->text().toDouble())  // Z
            ),
            Motion<Scalar>(
                Vec3(
                    table.item(8, 0)->text().toDouble(),  // X
                    table.item(9, 0)->text().toDouble(),  // Y
                    table.item(10, 0)->text().toDouble()), // Z
                Vec3(
                    table.item(11, 0)->text().toDouble(), // X
                    table.item(12, 0)->text().toDouble(), // Y
                    table.item(13, 0)->text().toDouble()) // Z
            )
    );
}

////////////////////////////////////////////////////////////////////////////////

// instantiate templates, so we don't have to define the template in the header
template struct State<double>;
