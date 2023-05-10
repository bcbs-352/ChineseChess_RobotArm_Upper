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

    //ui->label_port_status->setFont(QFont("Microsoft YaHei", 10,16));


    // 机械臂控制相关
    curPos = cv::Vec3f(412.f,0.f,0.f);
    curStep = cv::Vec3f(0.f,0.f,0.f);
    armSyncTimer = new QTimer(this);
    connect(armSyncTimer, &QTimer::timeout, this, &MainWindow::UpdateArmStep);
    connect(ui->btn_oper_arm, &QPushButton::clicked, this, &MainWindow::ConnectArm);
    connect(ui->btn_open_pump, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
            SendSerialPortMessage("M3 ");
    });
    connect(ui->btn_close_pump, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
            SendSerialPortMessage("M4 ");
    });
    connect(ui->btn_enable_motor, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
            SendSerialPortMessage("M6 ");
    });
    connect(ui->btn_unable_motor, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
            SendSerialPortMessage("M7 ");
    });
    connect(ui->btn_get_curpos, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
            SendSerialPortMessage("M0 ");
    });
    connect(ui->btn_set_curpos, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable())
        {
            QString msg = "M1 X" + QString::number(ui->doubleSpinBox_curpos_x->value()) + " Y" +  QString::number(ui->doubleSpinBox_curpos_y->value()) + " Z" + QString::number(ui->doubleSpinBox_curpos_z->value());
            SendSerialPortMessage(msg.toUtf8());
        }
    });
    connect(ui->btn_set_tarpos, &QPushButton::clicked, this, [&](){
        if(my_serial->isWritable()){
            QString msg = "M2 X" + QString::number(ui->doubleSpinBox_tarpos_x->value()) + " Y" +  QString::number(ui->doubleSpinBox_tarpos_y->value()) + " Z" + QString::number(ui->doubleSpinBox_tarpos_z->value());
            SendSerialPortMessage(msg.toUtf8());
        }
    });
    connect(ui->btn_calc_pos2location, &QPushButton::clicked, this, [&](){
        ImgProcessing::CalsPos2Location(Vec2f(ui->doubleSpinBox_leftup_x->value(), ui->doubleSpinBox_leftup_y->value()),
                                        Vec2f(ui->doubleSpinBox_rightup_x->value(), ui->doubleSpinBox_rightup_y->value()),
                                        Vec2f(ui->doubleSpinBox_rightdown_x->value(), ui->doubleSpinBox_rightdown_y->value()));
    });
    connect(ui->btn_start_task_flow, &QPushButton::clicked, this, &MainWindow::ArmTaskFlow);

    // Socket相关绑定
    tcpSocket = new QTcpSocket();
    //connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(OnSocketReadyRead()));
    connect(ui->btn_open_socket, &QPushButton::clicked, this, [&](){
        tcpSocket->connectToHost("127.0.0.1",8088);
        qDebug()<<"建立TcpSocket连接";
    });
    connect(this->tcpSocket, &QTcpSocket::readyRead, this, [=](){
        OnSocketReadyRead();
    });

    // 串口相关绑定
    my_serial=new QSerialPort(this);
    connect(ui->btn_refresh_serial, &QPushButton::clicked, this, &MainWindow::GetSerialPort);
    connect(ui->btn_open_port, &QPushButton::clicked, this, &MainWindow::OpenSerialPort);
    connect(ui->btn_close_port, &QPushButton::clicked, this, &MainWindow::CloseSerialPort);

    connect(ui->btn_stop_motor, &QPushButton::clicked, this, [&](){
        this->SendSerialPortMessageHex(SerialController::StopMotor(ui->cmb_motor_addr->currentIndex()));
    });
    connect(ui->btn_check_encoder, &QPushButton::clicked, this, [&](){
        this->SendSerialPortMessageHex(SerialController::CheckEncoder(ui->cmb_motor_addr->currentIndex()));
    });
    connect(ui->btn_send_subdiv, &QPushButton::clicked, this, [&](){
        this->SendSerialPortMessageHex(SerialController::SetMotorSubdiv(ui->cmb_motor_addr->currentIndex(),ui->cmb_subdiv->currentText().toInt()));
    });
    connect(ui->btn_motor_run, &QPushButton::clicked, this, [&](){
        this->SendSerialPortMessageHex(SerialController::StartMotor(ui->cmb_motor_addr->currentIndex(),ui->radioBtn_reverse->isChecked(),ui->cmb_max_speed->currentText().toInt()));
    });
    connect(ui->btn_motor_pluse, &QPushButton::clicked, this, [&](){
        this->SendSerialPortMessageHex(SerialController::SetMotorPos(ui->cmb_motor_addr->currentIndex(),ui->radioBtn_reverse->isChecked(),ui->cmb_max_speed->currentText().toInt(),ui->cmb_send_pluse->currentText().toInt()));
    });
    // 控制器相关绑定
    //gamePad = new QGamepad(0,this);
    connect(ui->btn_oper_controller, &QPushButton::clicked, this, &MainWindow::ConnectController);

    // 图像处理相关绑定
    connect(ui->btn_select_img_path, &QPushButton::clicked, this, &MainWindow::ReadImgFile);
    connect(ui->btn_test_img, &QPushButton::clicked, this, [=](){
        // 传入变量
        vector<double> cannyParams = {ui->SpinBox_min_canny->value(), ui->SpinBox_max_canny->value()};
        vector<double> houghCircleParams = {ui->SpinBox_bp->value(), ui->SpinBox_min_Dist->value(), ui->SpinBox_circle_threshold->value(),
                                            ui->SpinBox_min_R->value(), ui->SpinBox_max_R->value()};
        vector<Vec2f> cornerPoints = {Vec2f(ui->doubleSpinBox_leftup_x->value(), ui->doubleSpinBox_leftup_y->value()),
                                      Vec2f(ui->doubleSpinBox_rightup_x->value(), ui->doubleSpinBox_rightup_y->value()),
                                      Vec2f(ui->doubleSpinBox_rightdown_x->value(), ui->doubleSpinBox_rightdown_y->value())};
        vector<vector<int>> gridInfo = ImgProcessing::MainFunc(img, cannyParams, houghCircleParams);
        ImgProcessing::CalsPos2Location(cornerPoints[0], cornerPoints[1], cornerPoints[2]);

        cv::Mat cutImg = ImgProcessing::SplitChessBoard(img, cannyParams);

        QPixmap qPixmap(ui->label_broad_img->size());
        qPixmap.convertFromImage(ImgProcessing::Mat2QImage(cutImg));

        ui->label_broad_img->setPixmap(qPixmap.scaled(ui->label_broad_img->size()));

        QString fen = ImgProcessing::GetFenStr(gridInfo);
        qDebug() << "fen串:" << fen;
        ui->lineEdit_fen_str->clear();
        ui->lineEdit_fen_str->setText(fen);
    });
    // 摄像头相关
    updateTimer = new QTimer(this);
    connect(ui->btn_oper_cam, &QPushButton::clicked, this, [&](){
        if(cameraShow)
        {
            cap.release();
            updateTimer->stop();
            cameraShow=false;
            ui->btn_oper_cam->setText("打开相机");
        }else{
            cap.open(2);
            updateTimer->start(30);
            cameraShow=true;
            cap.set(cv::CAP_PROP_FRAME_WIDTH,W);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT,H);
            ui->btn_oper_cam->setText("关闭相机");
        }
    });
    connect(ui->btn_set_cam, &QPushButton::clicked, this, [&](){
        cap.set(cv::CAP_PROP_BRIGHTNESS, ui->spinBox_brightness->value());
        cap.set(cv::CAP_PROP_CONTRAST, ui->spinBox_contrast->value());
        cap.set(cv::CAP_PROP_HUE, ui->spinBox_hue->value());
        cap.set(cv::CAP_PROP_SATURATION, ui->spinBox_saturation->value());
        cap.set(cv::CAP_PROP_GAMMA, ui->spinBox_gamma->value());
    });
    connect(ui->spinBox_brightness, &QSpinBox::valueChanged, this, &MainWindow::ModifyCamera);
    connect(ui->spinBox_contrast, &QSpinBox::valueChanged, this, &MainWindow::ModifyCamera);
    connect(ui->spinBox_hue, &QSpinBox::valueChanged, this, &MainWindow::ModifyCamera);
    connect(ui->spinBox_saturation, &QSpinBox::valueChanged, this, &MainWindow::ModifyCamera);
    connect(ui->spinBox_gamma, &QSpinBox::valueChanged, this, &MainWindow::ModifyCamera);
    connect(updateTimer, &QTimer::timeout, this, &MainWindow::CameraUpdate);

    // 象棋引擎相关绑定
    connect(ui->btn_select_exe_path, &QPushButton::clicked, this, &MainWindow::SelectExeFile);
    connect(ui->btn_start_exe, &QPushButton::clicked, this, &MainWindow::StartExeProcess);
    connect(ui->btn_send_exe, &QPushButton::clicked, this, &MainWindow::SendExeCommand);
    connect(ui->btn_calc_go, &QPushButton::clicked, this, [&](){
        QString fen = ui->lineEdit_fen_str->text();
        if(fen.length()==0)
            return;
        QString side = ui->radioBtn_go_black->isChecked() ? "b" : "w";
        QString depth = QString::number(ui->spinBox_calc_depth->value());
        QString sendMsg = "position fen " + fen + " " + side + " - - 0 1\r\n" + "go depth " + depth + "\r\n";
        qDebug() << "发送局面信息:" +sendMsg;
        if(process->isWritable())
            process->write(sendMsg.toLocal8Bit());
        else
            qDebug() << "无法发送";
        ui->lineEdit_fen_str->clear();
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
    foreach(const QSerialPortInfo&info, QSerialPortInfo::availablePorts())
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
    if(message.startsWith("curPos"))
    {
        int x_index = message.indexOf("X"), y_index = message.indexOf("Y"), z_index = message.indexOf("Z");
        ui->doubleSpinBox_curpos_x->setValue(message.mid(x_index+1 ,y_index-x_index-1).toDouble());
        ui->doubleSpinBox_curpos_y->setValue(message.mid(y_index+1 ,z_index-y_index-1).toDouble());
        ui->doubleSpinBox_curpos_z->setValue(message.mid(z_index+1).toDouble());
    }
}

// 发送串口数据   已从char*改为QByteArray     char*还需要长度参数
void MainWindow::SendSerialPortMessageHex(QByteArray send) //char*send)
{
    qDebug()<<"串口发送:"<<SerialController::Str2Hex(send);
    my_serial->write(send);
    ui->plainTextEdit->insertPlainText("发送:\n"+SerialController::Str2Hex(send)+'\n');
    // free(send);     // malloc的char*记得手动释放
}

void MainWindow::SendSerialPortMessage(QByteArray send)
{
    qDebug()<<"串口发送:"<<send;
    my_serial->write(send);
    ui->plainTextEdit->insertPlainText("发送:\n"+send+'\n');
}

void MainWindow::Sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while(QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::TestFunc()
{
    qDebug() << "test1";
    Sleep(1000);
    qDebug() << "wake up";
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

    // 接收到数据时
    connect(process,&QProcess::readyReadStandardOutput,this,[=](){
        std::string msg = process->readAllStandardOutput().toStdString();
        //qDebug() << "接收数据:" << QString::fromStdString(msg);
        ui->plainTextEdit_recv_msg->appendPlainText(QString::fromStdString(msg));
        std::string::size_type pos = msg.find("bestmove");
        if(pos != msg.npos)
        {
            qDebug() << "下一步预测:" << QString::fromStdString(msg.substr(pos+9,4));
            ui->lineEdit_predict->setText(QString::fromStdString(msg.substr(pos+9,4)));
        }
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
    process->write("ucci\r\n");
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

// 设置摄像头参数:亮度、对比度、色调、饱和度、伽马
void MainWindow::ModifyCamera()
{
    if(!cap.isOpened())
        return;
    cap.set(cv::CAP_PROP_BRIGHTNESS, ui->spinBox_brightness->value());
    cap.set(cv::CAP_PROP_CONTRAST, ui->spinBox_contrast->value());
    cap.set(cv::CAP_PROP_HUE, ui->spinBox_hue->value());
    cap.set(cv::CAP_PROP_SATURATION, ui->spinBox_saturation->value());
    cap.set(cv::CAP_PROP_GAMMA, ui->spinBox_gamma->value());
}

void MainWindow::ConnectController()
{
    if(gamePad!=nullptr && gamePad->isConnected())
    {
        gamePad->disconnect();
        delete gamePad;
        //qDebug() << "连接状态" <<gamePad->isConnected();
        ui->btn_oper_controller->setText("连接控制器");
        return;
    }
    QLoggingCategory::setFilterRules(QStringLiteral("qt.gamepad.debug=true"));
    auto gamepads = QGamepadManager::instance()->connectedGamepads();
    if (gamepads.isEmpty()) {
        qDebug() << "找不到设备";
        return;
    }
    gamePad = new QGamepad(*gamepads.begin(), this);
    ui->btn_oper_controller->setText("断开控制器");
    connect(gamePad, &QGamepad::axisLeftXChanged, this, [&](double value){
        deltaPos[0] = value * 5;
    });
    connect(gamePad, &QGamepad::axisLeftYChanged, this, [&](double value){
        deltaPos[1] = -value * 5;
    });
    connect(gamePad, &QGamepad::buttonR2Changed, this, [&](double value){
        deltaPos[2] = value * 0.5;
    });
    connect(gamePad, &QGamepad::buttonL2Changed, this, [&](double value){
        deltaPos[2] = -value * 0.5;
    });
}

void MainWindow::ConnectArm()
{
    if(armSyncTimer->isActive())
    {
        armSyncTimer->stop();
        ui->btn_oper_arm->setText("同步机械臂");
        return;
    }
    if(!my_serial->isOpen())
    {
        qDebug() << "串口未打开, 无法发送数据同步";
        //return;
    }
    armSyncTimer->start(100);
    ui->btn_oper_arm->setText("取消同步");
}

void MainWindow::UpdateArmStep()
{
    cv::Vec3f newPos = curPos + deltaPos;
    QByteArray msg = SerialController::SetMotorRunToStep(newPos[0], newPos[1], newPos[2]);
    if(msg.length()<=0)
        return;
    curPos = newPos;
    qDebug() << "当前坐标:(" << newPos[0] << "," << newPos[1] << "," << newPos[2] << ")";
    ui->doubleSpinBox_curpos_x->setValue(newPos[0]);
    ui->doubleSpinBox_curpos_y->setValue(newPos[1]);
    ui->doubleSpinBox_curpos_z->setValue(newPos[2]);
    if(tcpSocket->isOpen() && tcpSocket->isWritable())
    {
        QByteArray m = ("setPos X" + QString::number(newPos[0]) + "Y" + QString::number(newPos[1]) + "Z" + QString::number(newPos[2]+130)).toUtf8() + "\n";
        qDebug() << m;
        tcpSocket->write(m);
    }
    if(my_serial->isWritable())
    {
        SendSerialPortMessage(msg);
    }
}

void MainWindow::SetArmPos()
{

}

void MainWindow::SetValveStatus()
{

}



void MainWindow::ArmTaskFlow()
{
    QString predict = ui->lineEdit_predict->text();
    if(predict.length() != 4)
    {
        ui->plainTextEdit_task_flow->appendPlainText("预测结果错误\n");
        return;
    }
    int x0_index = (int)(predict.at(0).toLatin1() - 'a'), x1_index = (int)(predict.at(2).toLatin1() - 'a');
    int y0_index = 9 - (int)(predict.at(1).toLatin1() - '0'), y1_index = 9 - (int)(predict.at(3).toLatin1() - '0');
    float high = ui->doubleSpinBox_high_pos->value(), low = ui->doubleSpinBox_low_pos->value();
    qDebug() << "p0(" << x0_index <<"," << y0_index <<") p1("<< x1_index<<","<<y1_index<<")";
    int stepIndex = 0;
    ui->plainTextEdit_task_flow->appendPlainText("\n————New Task————\n");
    QString msg;
    // 先移走被吃的棋子
    if(ImgProcessing::gridChessLocation[y1_index][x1_index] != Vec2f(-1,-1))
    {
        qDebug() << predict.mid(2,2) << "的棋子被吃, 需先移走";
        Vec2f deadChessLocation = ImgProcessing::gridChessLocation[y1_index][x1_index];
        Vec2f outsideLocation = Vec2f(ui->doubleSpinBox_outsize_x->value(), ui->doubleSpinBox_outsize_y->value());
        msg = "M2 X" + QString::number(deadChessLocation[0],'f',1) + " Y" + QString::number(deadChessLocation[1],'f',1) + " Z" + QString::number(high);
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
        msg = "M2 X" + QString::number(deadChessLocation[0],'f',1) + " Y" + QString::number(deadChessLocation[1],'f',1) + " Z" + QString::number(low);
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
        msg = "M3 ";
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
        msg = "M2 X" + QString::number(deadChessLocation[0],'f',1) + " Y" + QString::number(deadChessLocation[1],'f',1) + " Z" + QString::number(high);
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
        msg = "M2 X" + QString::number(outsideLocation[0],'f',1) + " Y" + QString::number(outsideLocation[1],'f',1) + " Z" + QString::number(high);
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
        msg = "M4 ";
        SendSerialPortMessage(msg.toUtf8());Sleep(500);
    }
    Vec2f initLocation = Vec2f(ui->doubleSpinBox_initpos_x->value(), ui->doubleSpinBox_initpos_y->value());
    Vec2f chessLocation = ImgProcessing::gridChessLocation[y0_index][x0_index];
    Vec2f gridLocation = ImgProcessing::gridLocation[y0_index][x0_index];
    msg = "M2 X" + QString::number(chessLocation[0],'f',1) + " Y" + QString::number(chessLocation[1],'f',1) + " Z" + QString::number(high);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M2 X" + QString::number(chessLocation[0],'f',1) + " Y" + QString::number(chessLocation[1],'f',1) + " Z" + QString::number(low);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M3 ";
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M2 X" + QString::number(chessLocation[0],'f',1) + " Y" + QString::number(chessLocation[1],'f',1) + " Z" + QString::number(high);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M2 X" + QString::number(gridLocation[0],'f',1) + " Y" + QString::number(gridLocation[1],'f',1) + " Z" + QString::number(high);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M2 X" + QString::number(gridLocation[0],'f',1) + " Y" + QString::number(gridLocation[1],'f',1) + " Z" + QString::number(low);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M4 ";
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    msg = "M2 X" + QString::number(initLocation[0],'f',1) + " Y" + QString::number(initLocation[1],'f',1) + " Z" + QString::number(high);
    SendSerialPortMessage(msg.toUtf8());Sleep(500);
    qDebug() << "走棋完成";
}

