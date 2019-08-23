#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <vr_puppets/ui_vr_puppets.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <QWidget>
#include <QLabel>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/UDPSocket.hpp>
#include <thread>
#include <QScrollArea>
#include <QRadioButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QtWidgets/QCheckBox>
#include <mutex>

#endif

class VRPuppets
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    VRPuppets();
    ~VRPuppets(){
        system("rosnode kill serial_node");
        if(udp_thread->joinable())
            udp_thread->join();
    }

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    void plotData();
    void rescale();
    void sendCommand();
    void controlModeChanged();
    void allToPosition();
    void allToVelocity();
    void allToDisplacement();
    void sliderMoved();
    void sliderMovedAll();
    void stop();
    void sendMotorCommandLinearActuators();
    void serialNode();
    void zero();
    void newMotor();
private:
    void receiveStatusUDP();
    void updateMotorCommands();
    void MotorCommand( const roboy_middleware_msgs::MotorCommand::ConstPtr &msg);
    bool EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool ControlMode(roboy_middleware_msgs::ControlMode::Request &req, roboy_middleware_msgs::ControlMode::Response &res);
Q_SIGNALS:
    void new_data();
    void new_motor();
private:
    Ui::VRPuppets ui;
    QWidget *widget_;
    QWidget* motor_command_scrollarea;
    vector<QWidget*> widgets;
    map<int,QCheckBox*> check;
    map<int,QSlider*> sliders;
    QVector<double> time;
    int counter = 0;
    map<int,QVector<double>> motor_position, motor_velocity, motor_displacement, motor_force, motor_pwm;
    map<int,string> ip_address;
    map<int,QRadioButton*> active,pos,vel,dis;
    map<int,int> Kp,Ki,Kd;
    map<int,int> control_mode;
    int samples_per_plot = 300;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus, motor_command_sub;
    ros::Publisher motor_command, motor_status;
    ros::ServiceClient zero_srv;
    ros::ServiceServer e_stop_server, control_mode_srv;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    UDPSocketPtr udp, udp_command;
    boost::shared_ptr<std::thread> udp_thread;
    bool initialized = false;
    map<int,int> set_points;
    mutex mux;

    bool EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool e_stop = false;

};
