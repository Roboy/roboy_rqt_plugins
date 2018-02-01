#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_audio/ui_roboy_audio.h>
#include <roboy_communication_cognition/AudioData.h>
#include <roboy_communication_cognition/FreqPower.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <qcustomplot.h>

#endif

#define NUMBER_OF_MOTORS_PER_FPGA 14
#define NUMBER_OF_FPGAS 6
#define NUMBER_OF_FREQUENCIES 16

class RoboyAudio
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    RoboyAudio();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    void plotData();
private:
    void MicAudio(const roboy_communication_cognition::AudioData::ConstPtr &msg);
    void AvgPower(const roboy_communication_cognition::FreqPower::ConstPtr &msg);
Q_SIGNALS:
    void newData();
private:
    Ui::RoboyAudio ui;
    QWidget *widget_;
    QVector<double> ticks;
    QVector<double> time;
    int counter = 0;
    QVector<double> freqLineData;
    double maxPower = 0;
    int samples_per_plot = 300;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::yellow, Qt::lightGray};
    ros::NodeHandlePtr nh;
    ros::Subscriber micAudio;
    ros::Subscriber avgPower;
};
