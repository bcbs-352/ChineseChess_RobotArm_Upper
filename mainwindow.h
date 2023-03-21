#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include "SerialController.h"
#include "ImgProcessing.h"
#include "QGamepad"
#include "QFileDialog"
#include "QMessageBox"
#include "opencv.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void SendSerialPortMessage(QByteArray);  //char*);

private:
    Ui::MainWindow *ui;
    QSerialPort*my_serial;
    cv::Mat img;

private slots:
    void GetSerialPort();
    void OpenSerialPort();
    void CloseSerialPort();
    void GetSerialPortMessage();

    void ReadImgFile();
};
#endif // MAINWINDOW_H
