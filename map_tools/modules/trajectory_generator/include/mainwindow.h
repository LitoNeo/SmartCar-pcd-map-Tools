#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include "traj_extract.h"
#include "../../utils/common.h"
#include <thread>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
} // namespace Ui
QT_END_NAMESPACE

using smartcar::tools::Traj_Extractor;
using smartcar::utils::traj_info;
using smartcar::utils::traj_type;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_input_folder_clicked();

    void on_pushButton_output_folder_clicked();

    void on_pushButton_start_clicked();

    void on_pushButton_create_clicked();

    void on_pushButton_del_one_point_clicked();

    void on_pushButton_finish_clicked();

    void on_horizontalSlider_weight_data_sliderMoved(int position);

    void on_horizontalSlider_weight_smooth_sliderMoved(int position);

    void on_horizontalSlider_tolerance_sliderMoved(int position);

    void on_weight_value_changed();

private:
    Ui::MainWindow *ui;
    std::string input_folder;
    std::string output_folder;
    Traj_Extractor traj_extractor = Traj_Extractor::getInstance();
    std::thread _th;
    bool is_start;

    void send_message_info(std::string str);
    void send_message_warn(std::string str);
    void send_message_error(std::string str);
    bool check_self_params();
};
#endif // MAINWINDOW_H
