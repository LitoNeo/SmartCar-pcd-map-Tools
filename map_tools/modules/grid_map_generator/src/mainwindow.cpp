#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_pushButton_input_clicked() {
  QString filename =
      QFileDialog::getExistingDirectory(this, tr("Select Input folder or file"),
                                        QCoreApplication::applicationDirPath());
  if (filename.isEmpty()) {
    return;
  }

  _input_folder = filename.toStdString();
  ui->lineEdit_input->setText(filename);
}

void MainWindow::on_pushButton_output_clicked() {
  QString filename = QFileDialog::getExistingDirectory(
      this, tr("Open Rosbag"), QCoreApplication::applicationDirPath());
  if (filename.isEmpty()) {
    return;
  }

  _output_folder = filename.toStdString();
  ui->lineEdit_output->setText(filename);
}

bool MainWindow::run_pcd_divider() {
  bool flag = divider.run();
  send_message_info("Finished!");
  ui->pushButton_run->setEnabled(true);
  return flag;
}

void MainWindow::on_pushButton_run_clicked() {
  grid_size = ui->lineEdit_gridSize->text().toDouble();
  voxel_size = ui->lineEdit_voxelSize->text().toDouble();
  if (!check_self_params())
    return;
  divider.setInFolder(_input_folder);
  divider.setOutFolder(_output_folder);
  divider.setGridSize(grid_size);
  divider.setVoxelSize(voxel_size);
  ui->pushButton_run->setEnabled(false);
  // 使用QTimer::singleShot()是为了防止button->setEnable(false)失效，至于是为什么，还不知道
  // 相当于起了一个新线程？
  QTimer::singleShot(100, this, SLOT(run_pcd_divider()));
}

bool MainWindow::check_self_params() {
  if (_input_folder == "" || !QDir(tr(_input_folder.c_str())).exists()) {
    QMessageBox::critical(this, tr("WARNING"),
                          tr("Input Folder Error, please check."));
    send_message_error("Input Folder error.");
    return false;
  }
  send_message_info("Input folder: " + _input_folder);

  if (_output_folder == "" || !QDir(tr(_output_folder.c_str())).exists()) {
    QMessageBox::critical(this, tr("WARNING"),
                          tr("Output Folder Error, please check."));
    send_message_error("Output Folder error.");
    return false;
  }
  send_message_info("Output folder: " + _output_folder);

  if (grid_size < 1.0) {
    QMessageBox::critical(this, tr("WARNING"),
                          tr("grid_size should be in range (1.0, 10000.0)"));
    return false;
  }
  char str[80];
  sprintf(str, "Will aplly grid divider with grid_size = %.2lf", grid_size);
  send_message_info(str);

  if (voxel_size < 0.2) {
    //        QMessageBox::warning(this, tr("WARNING"), tr("voxel_size should be
    //        in range (0.2, 5.0)"));
    send_message_warn("Will not apply voxel filter.");
    return true;
  }
  sprintf(str, "Will aplly voxel filter with voxel_size = %.2lf", voxel_size);
  send_message_info(str);
  return true;
}

// receive
void MainWindow::send_message_info(std::string str) {
  QString text = tr(str.c_str());
  ui->plainTextEdit->appendPlainText(text);
}

void MainWindow::send_message_warn(std::string str) {
  QString text = tr(str.c_str());
  ui->plainTextEdit->appendPlainText(text);
}

void MainWindow::send_message_error(std::string str) {
  QString text = tr(str.c_str());
  ui->plainTextEdit->appendPlainText(text);
}
