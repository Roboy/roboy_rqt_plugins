#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <msj_platform_rqt/ui_msj_platform_rqt.h>
#include <roboy_simulation_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPicture>
#include <QPainter>
#include <QtWidgets/QCheckBox>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <common_utilities/rviz_visualization.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <stdio.h>
#include <ros/package.h>
#include <qcustomplot.h>
#include <thread>
#include <common_utilities/UDPSocket.hpp>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#endif

#define NUMBER_OF_MOTORS 8
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/4096.0*2.0*M_PI)*(2.0*M_PI*0.0045))
#define INF 500000

using namespace Eigen;
using namespace std;

class MSJPlatformRQT
        : public rqt_gui_cpp::Plugin, rviz_visualization {
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
    void plotJointState();
    void rescale();
    void rescaleMagneticSensors();
    void plotMotorChanged();
    void toggleAll();
    void fpgaChanged(int fpga);
    void motorPosChanged(int pos);
    void stopButtonClicked();
    void zeroClicked();
    void showMagneticField();
    void clearMagneticField();
    void zeroPose();
    void calibrateSystem();
    void measureJointLimits();
private:
    /**
     * Reads a yaml joint limits file
     * @param filepath to config
     * @return success
     */
    bool readJointLimits(const string &filepath);
    /**
     * Writes a yaml motor config file
     * @param filepath
     * @return success
     */
    bool writeJointLimits(const string &filepath);
    void gridMap();
    bool fileExists(const string &filepath);
    void MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg);
    void MagneticSensor(const roboy_middleware_msgs::MagneticSensor::ConstPtr &msg);
    void JointState(const sensor_msgs::JointState::ConstPtr &msg);
    long closest(QVector<double> const& vec, double value);

    void receivePose();
    int pnpoly(QVector<double> limits_x, QVector<double> limits_y, double testx, double testy);
Q_SIGNALS:
    void newData();
    void newJointState();
private:
    Ui::MSJPlatformRQT ui;
    QWidget *widget_;

    QVector<double> time, time_sensor;
    QVector<double> limits[3];
    int counter = 0;
    QVector<double> motorData[NUMBER_OF_FPGAS+1][NUMBER_OF_MOTORS][3], sensorData[3][3], q[3];
    QVector<Matrix4d> poseData;
    Quaterniond zero_rot;
    int imu_state[4];
    bool motorConnected[NUMBER_OF_FPGAS+1][NUMBER_OF_MOTORS], plotMotor[NUMBER_OF_MOTORS], show_magnetic_field = false;
    int samples_per_plot = 300;
    QColor color_pallette[16] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray, Qt::green, Qt::cyan};
    ros::NodeHandlePtr nh;
    ros::Subscriber motorStatus, magneticSensor, joint_state;
    ros::Publisher motorCommand, sphere_axis0, sphere_axis1, sphere_axis2;
    ros::ServiceClient emergencyStop, zero;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<std::thread> grid_thread, udp_thread;
    UDPSocketPtr udp;
    mutex mux;
};
