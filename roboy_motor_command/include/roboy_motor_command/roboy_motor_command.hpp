#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_motor_command/ui_roboy_motor_command.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <QWidget>
#include <QtQuick/QQuickView>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QScrollArea>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QLabel>
#include <map>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <std_srvs/SetBool.h>

#endif

using namespace std;

class RoboyMotorCommand
        : public rqt_gui_cpp::Plugin, MotorConfig {
    Q_OBJECT
public:
    RoboyMotorCommand();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
public Q_SLOTS:
    void stopButtonAllClicked();
    void setPointChanged();
    void setPointChangedSlider();
    void setPointAllChanged();
    void setPointAllChangedSlider();
    void controlModeChanged();
    void update_config();
    void loadMotorConfig();
    void addFpgaWidgets(int fpga);
private:
    Ui::RoboyMotorCommand ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    ros::Publisher motorCommand;
    ros::ServiceClient motorControl[6], motorConfig[6], emergencyStop[6];
    boost::shared_ptr<ros::AsyncSpinner> spinner;
private:
    bool stopButton;
    vector<double> setpoint;
    vector<int> control_mode;
    int total_number_of_motors = 0, number_of_fpgas = 6;
    map<int,QRadioButton*> pos, vel, dis, force, cur, dir;
    map<int,QSlider*> setpoint_slider_widget;
    QSlider *setpoint_slider_widget_all;
    map<int,QLineEdit*> setpoint_widget;
    QLineEdit *setpoint_widget_all;
    QLineEdit* scale;
    QWidget* motor_command_scrollarea;
    vector<QWidget*> widgets;
};
