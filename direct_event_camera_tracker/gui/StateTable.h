#ifndef STATETABLE_H_0YAVJ3NP
#define STATETABLE_H_0YAVJ3NP

#include <iostream>

#include "point_yaml.h"

#include <QAction>
#include <QIcon>
#include <QTableWidget>
#include <QClipboard>
#include <QApplication>
#include <QFileDialog>

class StateTable : public QTableWidget
{
    Q_OBJECT
public:
    StateTable(QWidget* parent = nullptr);

    Statef getState() const { return Statef::from_table(*this); }
    void setState(const Statef& state) { state.set_table(*this); }

signals:
    void goto_time(ros::Time t);

public slots:
    void on_goto_time();
    void on_copy();
    void on_copy_csv();
    void on_paste();
    void on_save();
    void on_load();

private:

    QAction action_goto_time;
    QAction action_copy, action_copy_csv, action_paste, action_save, action_load;
};


#endif /* end of include guard: STATETABLE_H_0YAVJ3NP */
