#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <msj_platform_rqt/ui_msj_platform_rqt.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <QWidget>
#include <QtWidgets/QCheckBox>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>

#endif

#define NUMBER_OF_MOTORS 8
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/4096.0*2.0*M_PI)*(2.0*M_PI*0.0045))

class MSJPlatformRQT
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    MSJPlatformRQT();

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
    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg);
Q_SIGNALS:
    void newData();
private:
    Ui::MSJPlatformRQT ui;
    QWidget *widget_;

    QVector<double> time;
    int counter = 0;
    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS][3];
    bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS], plotMotor[NUMBER_OF_MOTORS];
    int samples_per_plot = 300;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
};
