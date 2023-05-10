#include "SerialController.h"

#define PI 3.1415926535897932384626433832795
#define RAG2DEG 180 / PI
const float L1 = 212, L2 = 200, s = 8; // 主臂-L1, 副臂-L2, 丝杠螺距-s
const float zStep = 100.f * 4 / 3;     // 200/8*4, z向移动1mm所需步数
const float upperArmStep = 32.0f;      // 200/360*16*(72/20), 主臂转动1度所需步数
const float foreArmStep = 51.77f;      // 200/360*16*(62/20)*(62/33), 副臂转动1度所需步数
/*
void SerialController::RefreshPort()
{
}

bool SerialController::OpenSerialPort(QString name, int baud, QSerialPort::DataBits dataBits, QSerialPort::StopBits stopBits)
{
    my_serial->setPortName(name);
    my_serial->setBaudRate(baud);
    my_serial->setDataBits(dataBits);
    my_serial->setStopBits(stopBits);
    my_serial->open(QIODevice::ReadWrite);
    if(my_serial->isOpen()){
        return true;
    }
    return false;
}
*/

// 停止电机 e_ f7 tCHK
QByteArray SerialController::StopMotor(int motorIndex)
{
    int len=3;
    //char*ret=(char*)malloc((len+1)*sizeof(char));
    QByteArray ret;
    ret.resize(len);

    ret[0]=0xe0+motorIndex;
    ret[1]=0xf7;
    GetCHK(ret,len);
    //qDebug()<<Qt::hex<<(uint8_t)ret[0]<<(uint8_t)ret[1]<<(uint8_t)ret[2];

    return ret;
}

QByteArray SerialController::SetMotorPos(int motorIndex, bool direction, int speed, int pluse)
{

}

// 校验编码器 e_ 0f 00 tCHK
QByteArray SerialController::CheckEncoder(int motorIndex)
{
    int len=4;
    QByteArray ret;
    ret.resize(len);

    ret[0]=0xe0+motorIndex;
    ret[1]=0x80;
    ret[2]=0x00;
    GetCHK(ret,len);
    return ret;
}

// 设置任意细分 e_ 84 MS tCHK
QByteArray SerialController::SetMotorSubdiv(int motorIndex, int subdiv)
{
    int len=4;
    QByteArray ret;
    ret.resize(len);
    ret[0]=0xe0+motorIndex;
    ret[1]=0x84;
    ret[2]=static_cast<uint8_t>(subdiv);
    GetCHK(ret,len);
    return ret;
}

QByteArray SerialController::SetArmRunToPosition(float x, float y, float z)
{
    QString ret;
    ret = "M2 X" + QString::number(x) + " Y" + QString::number(y) + " Z" + QString::number(z);
    return ret.toUtf8();
}

QByteArray SerialController::SetMotorRunToStep(float x, float y, float z)
{
    QString ret;
    float theta1, theta2, theta3;
    float alpha = atan2(y, x);
    float cosTheta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    float sinTheta2 = sqrt(1 - cosTheta2 * cosTheta2); // 逆时针为正

    theta1 = alpha - atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);
    theta2 = atan2(sinTheta2, cosTheta2);
    theta3 = 2 * PI * z / s;
    if (isnan(theta1) || isnan(theta2) || isinf(theta1) || isinf(theta2))
    {
        qDebug() << "超出边界";
        return ret.toUtf8();
    }
    int aStep = upperArmStep * theta1 * RAG2DEG;
    int bStep = (theta1 * 33 / 62 + theta2) * foreArmStep * RAG2DEG; // 主臂转动θ度时，主臂与副臂之间的夹角会改变(逆时针方向θ*r1/r2)
    int cStep = zStep * theta3;
    ret = "M5 X" + QString::number(aStep) + " Y" + QString::number(bStep) + " Z" + QString::number(cStep);
    return ret.toUtf8();
}

// 更新传入字符串校验位的值
void SerialController::GetCHK(QByteArray&str,int len){
    str[len-1]=0x00;
    for(int i=0;i<len-1;++i)
    {
        str[len-1]+=str[i];
    }
}

// 0~15的uint8转为char字符
char SerialController::Dec2Hex(uint8_t num)
{
    if(num>=0 && num <16)
    {
        if(num>=10)
        {
            return num-10+'a';
        }
        return num+'0';
    }
    return '?';
}

// 字节流转HEX形式的字符串 0xe1 转为'e'+'1'
QString SerialController::Str2Hex(const QByteArray&str)
{
    QString ret;
    ret.resize(str.size()*3-1);
    for(int i=0;i<ret.length();i+=3)
    {
        ret[i]=Dec2Hex((uint8_t)str[i/3]/16);
        ret[i+1]=Dec2Hex((uint8_t)str[i/3]%16);
        if(i+2<ret.length())
            ret[i+2]=' ';
    }
    return ret;
}

// 电机以一定的速度进行正/反转 e_ f6 ds tCHK
QByteArray SerialController::StartMotor(int motorIndex, bool reverse, int speed)
{
    int len=4;
    QByteArray ret;
    ret.resize(len);
    ret[0]=0xe0+motorIndex;
    ret[1]=0xf6;
    ret[2]=(reverse) ? 0x80 : 0x00;
    ret[2]+=static_cast<uint8_t>(speed);
    GetCHK(ret,len);
    return ret;
}

