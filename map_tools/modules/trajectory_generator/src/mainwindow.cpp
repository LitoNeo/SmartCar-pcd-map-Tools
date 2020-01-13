#include "../include/mainwindow.h"
#include "./ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->horizontalSlider_weight_data, SIGNAL(sliderReleased()), this, SLOT(on_weight_value_changed()));
    connect(ui->horizontalSlider_weight_smooth, SIGNAL(sliderReleased()), this, SLOT(on_weight_value_changed()));
    connect(ui->horizontalSlider_tolerance, SIGNAL(sliderReleased()), this, SLOT(on_weight_value_changed()));

    ui->pushButton_create->setEnabled(false);
    ui->pushButton_del_one_point->setEnabled(false);
    ui->pushButton_finish->setEnabled(false);

    input_folder = "";
    output_folder = "";
    is_start = false;
}

MainWindow::~MainWindow()
{
    delete ui;
    if (_th.joinable())
    {
        traj_extractor.shutdown();
        _th.join();
    }
}

void MainWindow::on_pushButton_input_folder_clicked()
{
    QString filename = QFileDialog::getExistingDirectory(this, tr("Select input folder with pcds"), QCoreApplication::applicationDirPath());
    if (filename.isEmpty())
        return;
    this->input_folder = filename.toStdString();
    ui->lineEdit_input_folder->setText(filename);
}

void MainWindow::on_pushButton_output_folder_clicked()
{
    QString filename = QFileDialog::getExistingDirectory(this, tr("Select input folder with pcds"), QCoreApplication::applicationDirPath());
    if (filename.isEmpty())
        return;
    this->output_folder = filename.toStdString();
    ui->lineEdit_output_folder->setText(filename);
}

bool MainWindow::check_self_params()
{
    if (this->input_folder == "" || !QDir(tr(this->input_folder.c_str())).exists())
    {
        send_message_error("Input Folder error, please check it.");
        return false;
    }
    send_message_info("Input folder: " + this->input_folder);

    if (this->output_folder == "" || !QDir(tr(this->output_folder.c_str())).exists())
    {
        send_message_error("Input Folder error, please check it.");
        return false;
    }
    send_message_info("Output folder: " + this->output_folder);
    return true;
}

void MainWindow::on_pushButton_start_clicked()
{
    if (!check_self_params())
        return;
    if (is_start)
    {
        traj_extractor.reload_map();
        return;
    }
    is_start = true;
    // traj_extractor.start(input_folder, output_folder);
    _th = std::thread(boost::bind(&Traj_Extractor::launch_thread, &traj_extractor, input_folder, output_folder));
    ui->pushButton_create->setEnabled(true);
    ui->pushButton_del_one_point->setEnabled(true);
    ui->pushButton_finish->setEnabled(true);
}

// receive
void MainWindow::send_message_info(std::string str)
{
    QString text = tr(str.c_str());
    ui->plainTextEdit->appendPlainText(text);
}

void MainWindow::send_message_warn(std::string str)
{
    QString text = tr(str.c_str());
    QMessageBox::warning(this, tr("WARNING"), tr(str.c_str()));
    ui->plainTextEdit->appendPlainText(text);
}

void MainWindow::send_message_error(std::string str)
{
    QString text = tr(str.c_str());
    QMessageBox::critical(this, tr("WARNING"), tr(str.c_str()));
    ui->plainTextEdit->appendPlainText(text);
}

void MainWindow::on_pushButton_create_clicked()
{
    traj_info _traj_info;

    // type
    std::string type = ui->comboBox_traj_type->currentText().toStdString();
    if (type == "lane")
        _traj_info.type = traj_type::LANE;
    else
        _traj_info.type = traj_type::CROSS;

    // id
    QString id = ui->lineEdit_traj_id->text();
    if (id == "")
    {
        send_message_warn("id not set !");
        return;
    }
    else
    {
        _traj_info.id = id.toInt();
    }

    // pre_ids
    QString pre_ids = ui->lineEdit_traj_pre_ids->text();
    if (pre_ids == "")
    {
        send_message_warn("pre_ids not set !");
        return;
    }
    else
    {
        QStringList arr = pre_ids.split(",");
        if (arr.isEmpty())
        {
            send_message_warn("use dot(,) to define ids");
            return;
        }
        for (QString s : arr)
            _traj_info.pre_ids.push_back(s.toInt());
    }

    // next_ids
    QString next_ids = ui->lineEdit_traj_next_ids->text();
    if (next_ids == "")
    {
        send_message_warn("next_ids not set");
        return;
    }
    else
    {
        QStringList arr = next_ids.split(",");
        if (arr.isEmpty())
        {
            send_message_warn("use dot(,) to define ids");
            return;
        }
        for (QString s : arr)
            _traj_info.next_ids.push_back(s.toInt());
    }

    // reverse
    std::string reverse_str = ui->comboBox_traj_reverse->currentText().toStdString();
    if (reverse_str == "true")
        _traj_info.reverse = true;
    else
        _traj_info.reverse = false;

    // conduct
    if (!this->traj_extractor.create_trajtory(_traj_info))
    {
        send_message_error("Unknown error: create trajectory failed !");
        return;
    }
}

void MainWindow::on_pushButton_del_one_point_clicked()
{
    if (!this->traj_extractor.delete_one_point())
    {
        this->send_message_warn("no point to delete !");
    }
}

void MainWindow::on_pushButton_finish_clicked()
{
    if (!this->traj_extractor.write_back())
    {
        this->send_message_error("Unknown Error happened when write file to disk.");
    }
    else
    {
        this->send_message_info("Tasks finished, please check " + this->output_folder);
    }
}

void MainWindow::on_horizontalSlider_weight_data_sliderMoved(int position)
{
    ui->lcdNumber_weight_data->display(position * 1.0 / 100.0);
}

void MainWindow::on_horizontalSlider_weight_smooth_sliderMoved(int position)
{
    ui->lcdNumber_weight_smooth->display(position * 1.0 / 100.0);
}

void MainWindow::on_horizontalSlider_tolerance_sliderMoved(int position)
{
    ui->lcdNumber_tolerance->display(position * 1.0 / 100.0);
}

void MainWindow::on_weight_value_changed()
{
    double weight_data = ui->horizontalSlider_weight_data->value();
    double weight_smooth = ui->horizontalSlider_weight_smooth->value();
    double tolerance = ui->horizontalSlider_tolerance->value();
    traj_extractor.reset_weight(weight_data * 1.0 / 100.0, weight_smooth * 1.0 / 100.0, tolerance * 1.0 / 100.0);
}