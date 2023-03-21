#ifndef SERIALCONTROLLER_H
#define SERIALCONTROLLER_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QWidget>
#include <QString>
#include <QDebug>


class SerialController{
/*
    void RefreshPort();
    bool OpenSerialPort(QString, int, QSerialPort::DataBits, QSerialPort::StopBits);
*/
private :
    QSerialPort*my_serial;

public:

    static char Dec2Hex(uint8_t);
    static QString Str2Hex(const QByteArray&);

    //static QByteArray SetDriverEnable(int motorIndex,bool enable);        // f3

    static QByteArray StartMotor(int motorIndex,bool direction, int speed); // f6
    static QByteArray StopMotor(int motorIndex);    // f7
    static QByteArray SetMotorPos(int motorIndex,bool direction,int speed,int pluse);   //fd

    static QByteArray SetMotorDefault(int motorIndex);      // 3f
    static QByteArray CheckEncoder(int motorIndex);         // 80
    static QByteArray SetMotorSubdiv(int motorIndex,int subdiv);    // 84
    static QByteArray SetMotorKp(int motorIndex,int Kp);    // a1
    static QByteArray SetMotorKi(int motorIndex,int Ki);    // a2
    static QByteArray SetMotorKd(int motorIndex,int Kd);    // a3
    static QByteArray SetMotorAcc(int motorIndex,int acc);  // a4
    static QByteArray SetMotorMaxTorque(int motorIndex,int torque); // a5

    static QByteArray GetMotorEncoder(int motorIndex);      // 30
    static QByteArray GetMotorCurrentPos(int motorIndex);   // 36
    static QByteArray GetMotorPluse(int motorIndex);        // 33
    static QByteArray GetMotorEnable(int motorIndex);       // 3a
    static QByteArray GetMotorPosErr(int motorIndex);       // 39

    static void GetCHK(QByteArray&,int);
};
#endif // SERIALCONTROLLER_H
