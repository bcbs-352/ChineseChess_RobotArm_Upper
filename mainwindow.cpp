#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "SerialController.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    my_serial=new QSerialPort(this);

    ui->label_port_status->setFont(QFont("Microsoft YaHei", 10,16));

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
        ImgProcessing::MainFunc(img);
    });
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
    qDebug()<<"图像尺寸"<<img.cols<<"×"<<img.rows;
}




