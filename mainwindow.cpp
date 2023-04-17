#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "SerialController.h"
#include <QProcess>

#define H 720
#define W 1280

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    my_serial=new QSerialPort(this);

    ui->label_port_status->setFont(QFont("Microsoft YaHei", 10,16));

    tcpSocket = new QTcpSocket();

    // Socket相关绑定
    //connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(OnSocketReadyRead()));
    connect(ui->btn_open_socket,&QPushButton::clicked,this,[&](){
        tcpSocket->connectToHost("127.0.0.1",8088);
        qDebug()<<"建立TcpSocket连接";
    });
    connect(this->tcpSocket,&QTcpSocket::readyRead,this,[=](){
        OnSocketReadyRead();
    });

    // 串口相关绑定
    connect(ui->btn_refresh_serial,&QPushButton::clicked,this,&MainWindow::GetSerialPort);
    connect(ui->btn_open_port,&QPushButton::clicked,this,&MainWindow::OpenSerialPort);
    connect(ui->btn_close_port,&QPushButton::clicked,this,&MainWindow::CloseSerialPort);

    connect(ui->btn_stop_motor,&QPushButton::clicked,this,[=](){
        this->SendSerialPortMessage(SerialController::StopMotor(ui->cmb_motor_addr->currentIndex()));
    });
    connect(ui->btn_check_encoder,&QPushButton::clicked,this,[=](){
        this->SendSerialPortMessage(SerialController::CheckEncoder(ui->cmb_motor_addr->currentIndex()));
    });
    connect(ui->btn_send_subdiv,&QPushButton::clicked,this,[=](){
        this->SendSerialPortMessage(SerialController::SetMotorSubdiv(ui->cmb_motor_addr->currentIndex(),ui->cmb_subdiv->currentText().toInt()));
    });
    connect(ui->btn_motor_run,&QPushButton::clicked,this,[=](){
        this->SendSerialPortMessage(SerialController::StartMotor(ui->cmb_motor_addr->currentIndex(),ui->radioBtn_reverse->isChecked(),ui->cmb_max_speed->currentText().toInt()));
    });
    connect(ui->btn_motor_pluse,&QPushButton::clicked,this,[=](){
        this->SendSerialPortMessage(SerialController::SetMotorPos(ui->cmb_motor_addr->currentIndex(),ui->radioBtn_reverse->isChecked(),ui->cmb_max_speed->currentText().toInt(),ui->cmb_send_pluse->currentText().toInt()));
    });


    // 图像处理相关绑定
    connect(ui->btn_select_img_path,&QPushButton::clicked,this,&MainWindow::ReadImgFile);
    connect(ui->btn_test_img,&QPushButton::clicked,this,[=](){
        //ImgProcessing::SplitChessBoard(img);
        // 传入变量
        vector<double> cannyParams = {ui->SpinBox_min_canny->value(), ui->SpinBox_max_canny->value()};
        vector<double> houghCircleParams = {ui->SpinBox_bp->value(), ui->SpinBox_min_Dist->value(), ui->SpinBox_circle_threshold->value(),
                                            ui->SpinBox_min_R->value(), ui->SpinBox_max_R->value()};
        vector<vector<int>> gridInfo = ImgProcessing::MainFunc(img, cannyParams, houghCircleParams);

        cv::Mat cutImg = ImgProcessing::SplitChessBoard(img, cannyParams);

        QPixmap qPixmap(ui->label_origin_img->size());
        qPixmap.convertFromImage(ImgProcessing::Mat2QImage(cutImg));

        ui->label_origin_img->setPixmap(qPixmap.scaled(ui->label_origin_img->size()));

        qDebug() << "fen串:" <<ImgProcessing::GetFenStr(gridInfo);
    });
    // 摄像头开关
    updateTimer = new QTimer(this);
    connect(ui->btn_open_cam,&QPushButton::clicked,this,[&](){
        cap.open(2);
        updateTimer->start(30);
        cameraShow=true;

        cap.set(cv::CAP_PROP_FRAME_WIDTH,W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,H);
        cap >> img;
        qDebug() << "打开摄像头,分辨率" << img.cols << "x" << img.rows;
    });
    connect(ui->btn_close_cam,&QPushButton::clicked,this,[&](){
        cap.release();
        updateTimer->stop();
        cameraShow=false;
    });
    connect(updateTimer,&QTimer::timeout,this,&MainWindow::CameraUpdate);

    // 象棋引擎相关绑定
    connect(ui->btn_select_exe_path,&QPushButton::clicked,this,&MainWindow::SelectExeFile);
    connect(ui->btn_start_exe,&QPushButton::clicked,this,&MainWindow::StartExeProcess);
    connect(ui->btn_send_exe,&QPushButton::clicked,this,&MainWindow::SendExeCommand);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// 刷新串口信息，获取串口数据
void MainWindow::GetSerialPort()
{
    qDebug()<<"刷新串口";
    ui->cmb_serial_port->clear();
    foreach(const QSerialPortInfo&info,QSerialPortInfo::availablePorts())
    {
        ui->cmb_serial_port->addItem(info.portName());
    }
}

// 开启串口
void MainWindow::OpenSerialPort()
{
    my_serial->setPortName(ui->cmb_serial_port->currentText());
    my_serial->setBaudRate(ui->cmb_baud->currentText().toInt());
    // 数据位默认8 停止位默认1 无流控制
    my_serial->setDataBits(QSerialPort::DataBits::Data8);
    my_serial->setStopBits(QSerialPort::StopBits::OneStop);
    my_serial->setFlowControl(QSerialPort::FlowControl::NoFlowControl);

    my_serial->open(QIODevice::ReadWrite);
    if(my_serial->isOpen())
    {
        ui->label_port_status->setText("打开成功");
        connect(my_serial,&QSerialPort::readyRead,this,&MainWindow::GetSerialPortMessage);
    }
    else
    {
        ui->label_port_status->setText("打开失败");
    }
}

void MainWindow::CloseSerialPort()
{
    my_serial->close();
    if(my_serial->isOpen()){
        ui->label_port_status->setText("串口未关闭");
    }else{
        ui->label_port_status->setText("串口已关闭");
    }
}

// 获取串口字节流，并打印到plainText框中
void MainWindow::GetSerialPortMessage()
{
    QByteArray message=my_serial->readAll();
    ui->plainTextEdit->insertPlainText("接收:\n"+message+'\n');
}

// 发送串口数据   已从char*改为QByteArray     char*还需要长度参数
void MainWindow::SendSerialPortMessage(QByteArray send) //char*send)
{
    qDebug()<<"串口发送:"<<SerialController::Str2Hex(send);
    my_serial->write(send);
    ui->plainTextEdit->insertPlainText("发送:\n"+SerialController::Str2Hex(send)+'\n');
    // free(send);     // malloc的char*记得手动释放
}

// 选择、读取、显示文件夹中的图片
void MainWindow::ReadImgFile()
{
    QString imgPath=QFileDialog::getOpenFileName(this,"选择图片","C:/Users/asus/Desktop",tr("Images(*.png *.jpg *.jpeg)"));
    qDebug()<<"打开文件"<<imgPath;
    ui->lineEdit_img_path->setText(imgPath);
    if(imgPath.isEmpty())
    {
        QMessageBox::warning(this,"Attention","图像路径错误",QMessageBox::Yes);
        return;
    }

    QPixmap qPixmap(ui->label_origin_img->size());
    qPixmap.convertFromImage(QImage(imgPath));

    ui->label_origin_img->setPixmap(qPixmap.scaled(ui->label_origin_img->size()));

    img = cv::imread(imgPath.toStdString());
    cv::resize(img,img,cv::Size(1280,720));
    cv::imshow("720p",img);
    qDebug()<<"图像尺寸"<<img.cols<<"×"<<img.rows;
}

void MainWindow::OnSocketReadyRead()
{
    QByteArray bytes = NULL;
    while(bytes.length() == 0)
    {
        while(tcpSocket->waitForReadyRead(300))
        {
            bytes.append((QByteArray)tcpSocket->readAll());
        }
        memcpy(imgBuffer,bytes,bytes.length());
        qDebug() << "数据长度:" << bytes.length();
    }

    vector<uchar>data(imgBuffer,imgBuffer+bytes.length());
    cv::Mat dst(cv::Size(W,H),CV_8UC3);
    dst = cv::imdecode(data,1);
    //cv::flip(dst,dst,-1);
//    for(int i=0;i<H;++i)
//    {
//        uchar*data=dst.ptr<uchar>(i);
//        uchar*pBuf=imgBuffer + (H-1-i)*W*3;
//        memcpy(data,pBuf,W*3);
//    }
    //cv::cvtColor(dst,dst,COLOR_RGB2BGR);
    img = dst;
    cv::imshow("img",img);
}

void MainWindow::SelectExeFile()
{
    QString exePath = QFileDialog::getOpenFileName(this,"选择象棋引擎","C:/Users/asus/Desktop",tr("exec(*.exe)"));
    qDebug() << "打开文件" << exePath;
    ui->lineEdit_exe_path->setText(exePath);
    if(exePath.isEmpty())
    {
        QMessageBox::warning(this,"Attention","图像路径错误",QMessageBox::Yes);
        return;
    }
}

void MainWindow::StartExeProcess()
{
    process = new QProcess();
    QString exePath = ui->lineEdit_exe_path->text();

    connect(process,&QProcess::readyReadStandardOutput,this,[=](){
        qDebug() << process->readAllStandardOutput();
    });

    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [=](int exitCode, QProcess::ExitStatus exitStatus)
    {
        qDebug() << "process finish." << exitCode << exitStatus;
    });


//    process->setCreateProcessArgumentsModifier([this](QProcess::CreateProcessArguments * args)
//    {
//        //下面这两行让后台exe弹出一个窗口
//        args->flags |=  CREATE_NEW_CONSOLE;
//        args->flags &= ~CREATE_NO_WINDOW;
//    });


    process->start(exePath);
//  process->start("cmd.exe");
//  process->write(exePath.toLocal8Bit());
    qDebug() << "process ID:" << process->processId();
}

void MainWindow::SendExeCommand()
{
    QString msg = ui->lineEdit_send_exe->text();
    ui->lineEdit_send_exe->clear();
    if(!process->isWritable())
    {
        qDebug() << "无法发送指令";
        return;
    }
    qDebug() << "发送指令:" << msg;
    process->write((msg+"\r\n").toLocal8Bit());
}

// 摄像头定时器调用,刷新并显示图像
void MainWindow::CameraUpdate()
{
    cap >> img;
    cv::Mat contourImg = img.clone();
    //cv::cvtColor(img,img,COLOR_BGR2RGB);
    cv::Rect rect(W/2-H*9/22,H/11,H*9/11,H*9/11);
    cv::rectangle(contourImg,rect,cv::Scalar(0,0,255),2);
    QPixmap qPixmap(ui->label_origin_img->size());
    qPixmap.convertFromImage(ImgProcessing::Mat2QImage(contourImg));
    ui->label_origin_img->setPixmap(qPixmap.scaled(ui->label_origin_img->size()));
}




