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
#include <QTcpSocket>
#include "opencv.hpp"
#include "imgcodecs/imgcodecs.hpp"
#include "QElapsedTimer"
#include <QProcess>
#include <QTimer>
#include <QLoggingCategory>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void SendSerialPortMessageHex(QByteArray);
    void SendSerialPortMessage(QByteArray);
    void Sleep(int msec);
    void TestFunc();
private:
    Ui::MainWindow *ui;
    QSerialPort *my_serial;
    cv::Mat img;


    QGamepad *gamePad;
    QTimer *armSyncTimer;
    cv::Vec3f curPos, curStep, deltaPos;

    QTcpSocket *tcpSocket;
    uchar imgBuffer[1280*720];

    QProcess *process;

    bool cameraShow = false;
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
    void ModifyCamera();

    void ConnectController();
    void ConnectArm();
    void UpdateArmStep();
    void SetArmPos();
    void SetValveStatus();

    void ArmTaskFlow();
};
#endif // MAINWINDOW_H
