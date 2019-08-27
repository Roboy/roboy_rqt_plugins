#include <shoulder_testbed/shoulder_testbed.hpp>
#include <QtWidgets/QCheckBox>

ShoulderTestbed::ShoulderTestbed()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("ShoulderTestbed");
}

void ShoulderTestbed::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>("/stepper_motor_shield/MotorCommand",1);

    zero_srv = nh->serviceClient<std_srvs::Empty>("/stepper_motor_shield/zero");

    QObject::connect(this, SIGNAL(new_data()), this, SLOT(plotData()));
    QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stop()));
    QObject::connect(ui.motor_0, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_1, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_2, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_3, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_4, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_5, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_6, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_7, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_8, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor_9, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.serial_node, SIGNAL(clicked()), this, SLOT(serialNode()));
    QObject::connect(ui.zero, SIGNAL(clicked()), this, SLOT(zero()));
    ui.stop->setStyleSheet("background-color: red");

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

    for (uint motor = 0; motor < 20; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor%16])); // %16 because we only have 16 colors :(
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.pwm_plot->addGraph();
        ui.pwm_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
    }
    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setLabel("ticks");
    ui.position_plot->replot();

    ui.velocity_plot->xAxis->setLabel("time[s]");
    ui.velocity_plot->yAxis->setLabel("ticks/s");
    ui.velocity_plot->replot();

    ui.displacement_plot->xAxis->setLabel("time[s]");
    ui.displacement_plot->yAxis->setLabel("ticks");
    ui.displacement_plot->replot();

    ui.pwm_plot->xAxis->setLabel("time[s]");
    ui.pwm_plot->yAxis->setLabel("[1]");
    ui.pwm_plot->replot();

    initialized = true;
}

void ShoulderTestbed::shutdownPlugin() {
    motorStatus.shutdown();
}

void ShoulderTestbed::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void ShoulderTestbed::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void ShoulderTestbed::plotData() {
    lock_guard<mutex> lock(mux);
    if(!ros::ok())
        return;
    for (auto m:ip_address) {
        ui.position_plot->graph(m.first)->setData(time, motor_position[m.first]);
        ui.velocity_plot->graph(m.first)->setData(time, motor_velocity[m.first]);
        ui.displacement_plot->graph(m.first)->setData(time, motor_displacement[m.first]);
        ui.pwm_plot->graph(m.first)->setData(time, motor_pwm[m.first]);
    }

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.displacement_plot->xAxis->rescale();
    ui.pwm_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.displacement_plot->replot();
    ui.pwm_plot->replot();
}

void ShoulderTestbed::rescale(){
    double minima[NUMBER_OF_MOTORS_PER_FPGA][4], maxima[NUMBER_OF_MOTORS_PER_FPGA][4];
    uint minimal_motor[4] = {0,0,0,0}, maximal_motor[4] = {0,0,0,0};
    map<int,QVector<double>> *motorData[4];
    motorData[0] = &motor_position;
    motorData[1] = &motor_velocity;
    motorData[2] = &motor_displacement;
    motorData[3] = &motor_pwm;
    for(uint type=0;type<4;type++) {
        for (auto m:ip_address) {
            minima[m.first][type] = 0;
            maxima[m.first][type] = 0;
            for (auto val:motorData[type]->at(m.first)) {
                if (val < minima[m.first][type])
                    minima[m.first][type] = val;
                if (val > maxima[m.first][type])
                    maxima[m.first][type] = val;
            }
        }

        for (auto m:ip_address) {
            if (minima[m.first][type] <= minima[minimal_motor[type]][type] && check[m.first]->isChecked())
                minimal_motor[type] = m.first;
            if (maxima[m.first][type] <= maxima[maximal_motor[type]][type] && check[m.first]->isChecked())
                maximal_motor[type] = m.first;
        }
    }

    for (auto m:ip_address) {
        if (minimal_motor[0] == m.first||maximal_motor[0] == m.first)
            ui.position_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[1] == m.first||maximal_motor[1] == m.first)
            ui.velocity_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[2] == m.first||maximal_motor[2] == m.first)
            ui.displacement_plot->graph(m.first)->rescaleAxes();
        if (minimal_motor[3] == m.first||maximal_motor[3] == m.first)
            ui.pwm_plot->graph(m.first)->rescaleAxes();
    }

    for (auto m:ip_address) {
        if (minimal_motor[0] != m.first||maximal_motor[0] != m.first)
            ui.position_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[1] != m.first||maximal_motor[1] != m.first)
            ui.velocity_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[2] != m.first||maximal_motor[2] != m.first)
            ui.displacement_plot->graph(m.first)->rescaleAxes(true);
        if (minimal_motor[3] != m.first||maximal_motor[3] != m.first)
            ui.pwm_plot->graph(m.first)->rescaleAxes(true);
    }
}

void ShoulderTestbed::sendMotorCommandLinearActuators(){
    roboy_middleware_msgs::MotorCommand msg;
    msg.id = 68;
    msg.motors = {0,1,2,3,4,5,6,7,8,9};
    msg.set_points = {(float)ui.motor_0->value(),
                      (float)ui.motor_1->value(),
                      (float)ui.motor_2->value(),
                      (float)ui.motor_3->value(),
                      (float)ui.motor_4->value(),
                      (float)ui.motor_5->value(),
                      (float)ui.motor_6->value(),
                      (float)ui.motor_7->value(),
                      (float)ui.motor_8->value(),
                      (float)ui.motor_9->value()};
    motor_command.publish(msg);
}

void ShoulderTestbed::serialNode(){
    system("rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0&");
}

bool ShoulderTestbed::EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data == 1) {
        ROS_INFO("M3-Emergency stop service called.");
        ui.stop->setChecked(true);
        res.success = true;
        res.message = "Emergency stop service called";
    } else {
        ROS_INFO("Resuming normal operation.");
        ui.stop->setChecked(false);
        res.success = true;
        res.message = "Resuming normal operation.";
    }
    return true;
}

void ShoulderTestbed::zero(){
    std_srvs::Empty msg;
    zero_srv.call(msg);
}

PLUGINLIB_EXPORT_CLASS(ShoulderTestbed, rqt_gui_cpp::Plugin)
