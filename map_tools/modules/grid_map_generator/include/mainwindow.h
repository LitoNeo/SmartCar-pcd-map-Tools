#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "pcd_grid_divider.h"
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <string>
#include "macro.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

// declear singleton
public:                              
    static MainWindow* instance() {
        static MainWindow instance;             
        return &instance;                      
    }  

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_input_clicked();

    void on_pushButton_output_clicked();

    void on_pushButton_run_clicked();

    bool check_self_params();

    bool run_pcd_divider();

public:
    void send_message_info(std::string str);

    void send_message_warn(std::string str);

    void send_message_error(std::string str);

private:
    Ui::MainWindow *ui;
    std::string _input_folder;
    std::string _output_folder;
    double grid_size;
    double voxel_size;

    MAP_TOOLS::pcd_grid_divider divider;
};
#endif // MAINWINDOW_H
