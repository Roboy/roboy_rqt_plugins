#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_motor_status/ui_roboy_motor_status.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>

#endif

class RoboyMotorStatus
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    RoboyMotorStatus();

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
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
Q_SIGNALS:
    void newData();
private:
    Ui::RoboyMotorStatus ui;
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
};
