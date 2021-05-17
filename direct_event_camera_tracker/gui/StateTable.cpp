#include "StateTable.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

StateTable::StateTable(QWidget* parent)
    : QTableWidget(parent),
    action_goto_time(QIcon::fromTheme("go-jump"), "go to time", this),
    action_copy (QIcon::fromTheme("edit-copy"), "copy", this),
    action_copy_csv(QIcon::fromTheme("edit-copy"), "copy as CSV", this),
    action_paste(QIcon::fromTheme("edit-paste"), "paste", this),
    action_save (QIcon::fromTheme("document-save"), "export", this),
    action_load (QIcon::fromTheme("document-open"), "import", this)
{
    Statef::init_table(*this);

    addAction(&action_goto_time);
    addAction(&action_copy);
    addAction(&action_copy_csv);
    addAction(&action_paste);
    addAction(&action_save);
    addAction(&action_load);

    connect(&action_goto_time, &QAction::triggered, this, &StateTable::on_goto_time);
    connect(&action_copy,  &QAction::triggered, this, &StateTable::on_copy);
    connect(&action_copy_csv,  &QAction::triggered, this, &StateTable::on_copy_csv);
    connect(&action_paste, &QAction::triggered, this, &StateTable::on_paste);
    connect(&action_save,  &QAction::triggered, this, &StateTable::on_save);
    connect(&action_load,  &QAction::triggered, this, &StateTable::on_load);
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_goto_time()
{
    emit goto_time(getState().stamp);
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_copy()
{
    YAML::Node n; n = getState();
    std::stringstream s; s << n;

    QApplication::clipboard()->setText(QString::fromStdString(s.str()));
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_copy_csv()
{
    Statef state = getState();

    std::stringstream s;
    for (size_t i = 0; i < Statef::get_attr_names().size(); i++) {
        s << state.attr(i);
        if (i+1 < Statef::get_attr_names().size()) {
            s << ",";
        } else {
            s << endl;
        }
    }


    QApplication::clipboard()->setText(QString::fromStdString(s.str()));
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_paste()
{
    const QString s = QApplication::clipboard()->text();

    const QStringList csv = s.split('\n', QString::SkipEmptyParts);

    if (csv.empty()) {
        cerr << "WARNING: pasting empty string? s='" << s.toStdString() << "'" << endl;
        return;
    }

    if (csv.length() < 3) {
        // text is most likely a CSV row and not YAML
        const QStringList row = csv[0].split(',');

        if (row.length() < 15) {
            cerr << "WARNING: Invalid CSV row, only " << row.length() << " columns" << endl;
            return;
        }

        Statef s(row);

        setState(s);
        return;
    } else {
        // treat it as YAML
        YAML::Node n = YAML::Load(s.toStdString());
        setState(n.as<Statef>());
    }
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_save()
{
    QString filename = QFileDialog::getSaveFileName(this, "Save Pose", "", "YAML (*.yaml)");

    if (filename.isEmpty()) {
        return;
    }

    if (!filename.endsWith(".yaml", Qt::CaseInsensitive)) {
        filename += ".yaml";
    }

    std::ofstream export_file(filename.toStdString());
    YAML::Node n; n = getState();
    export_file << n;
    export_file.close();
}

////////////////////////////////////////////////////////////////////////////////

void StateTable::on_load()
{
    QString filename = QFileDialog::getOpenFileName(this, "Load Pose", "", "YAML (*.yaml)");

    if (filename.isEmpty()) {
        return;
    }

    YAML::Node n = YAML::LoadFile(filename.toStdString());
    setState(n.as<Statef>());
}

////////////////////////////////////////////////////////////////////////////////

