﻿#ifndef MAINWINDOW_H
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
#include <QTcpSocket>
#include "opencv.hpp"
#include "imgcodecs/imgcodecs.hpp"
#include "QElapsedTimer"
#include <QProcess>
#include <QTimer>

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
    QSerialPort *my_serial;
    cv::Mat img;

    QGamepad gamePad;

    QTcpSocket *tcpSocket;
    uchar imgBuffer[1280*720];

    QProcess *process;

    bool cameraShow;
    cv::VideoCapture cap;
    QTimer *updateTimer;

private slots:
    void GetSerialPort();
    void OpenSerialPort();
    void CloseSerialPort();
    void GetSerialPortMessage();

    void ReadImgFile();
    void OnSocketReadyRead();

    void SelectExeFile();
    void StartExeProcess();
    void SendExeCommand();

    void CameraUpdate();
};
#endif // MAINWINDOW_H
