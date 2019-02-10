#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_arms/ui_roboy_arms.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <thread>         // std::thread

#endif

#define LEFT_PORT    8000    /* hard-coded port number */
#define RIGHT_PORT    8001   /* hard-coded port number */
#define BUFSIZE 2048
#define LEFT 0
#define RIGHT 1

class RoboyArms
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    RoboyArms();
    ~RoboyArms();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    void plotData();
    void rescale();
    void plotMotorChanged();
    void toggleAll();
    void fpgaChanged(int fpga);
private:
    void MotorStatus();
Q_SIGNALS:
    void newData();
private:
    Ui::RoboyArms ui;
    QWidget *widget_;

    QVector<double> time;
    int counter = 0;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
    bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA], plotMotor[NUMBER_OF_MOTORS_PER_FPGA];
    int samples_per_plot = 300;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<std::thread> arm_control_thread;

    struct sockaddr_in myaddr[2];    /* our address */
    struct sockaddr_in remaddr[2];    /* remote address */
    socklen_t addrlen = sizeof(remaddr[0]);        /* length of addresses */
    int recvlen[2];            /* # bytes received */
    int fd[2];                /* our socket */
    unsigned char buf[2][BUFSIZE];    /* receive buffer */
    bool arm_available[2];
};
