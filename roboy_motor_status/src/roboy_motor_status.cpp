#include <roboy_motor_status/roboy_motor_status.hpp>

RoboyMotorStatus::RoboyMotorStatus()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorStatus");
}

void RoboyMotorStatus::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.current_plot->addGraph();
        ui.current_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
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

    ui.current_plot->xAxis->setLabel("time[s]");
    ui.current_plot->yAxis->setLabel("mA");
    ui.current_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyMotorStatus::MotorStatus, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();
}

void RoboyMotorStatus::shutdownPlugin() {
    motorStatus.shutdown();
}

void RoboyMotorStatus::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorStatus::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorStatus::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving motor status");
    ros::Duration delta = (ros::Time::now()-start_time);
    time.push_back(delta.toSec());
    for (uint motor = 0; motor < msg->position.size(); motor++) {
        motorData[msg->id][motor][0].push_back(msg->position[motor]);
        motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
        motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
        motorData[msg->id][motor][3].push_back(msg->current[motor]);
        if (motorData[msg->id][motor][0].size() > samples_per_plot) {
            motorData[msg->id][motor][0].pop_front();
            motorData[msg->id][motor][1].pop_front();
            motorData[msg->id][motor][2].pop_front();
            motorData[msg->id][motor][3].pop_front();
        }
    }
    if (time.size() > samples_per_plot)
        time.pop_front();

    if ((counter++) % 20 == 0)
            Q_EMIT newData();
}

void RoboyMotorStatus::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.displacement_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
        ui.current_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][3]);

        if (motor == 0) {
            ui.position_plot->graph(motor)->rescaleAxes();
            ui.velocity_plot->graph(motor)->rescaleAxes();
            ui.displacement_plot->graph(motor)->rescaleAxes();
            ui.current_plot->graph(motor)->rescaleAxes();
        } else {
            ui.position_plot->graph(motor)->rescaleAxes(true);
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
            ui.displacement_plot->graph(motor)->rescaleAxes(true);
            ui.current_plot->graph(motor)->rescaleAxes(true);
        }
    }
    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.displacement_plot->replot();
    ui.current_plot->replot();
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_status, RoboyMotorStatus, RoboyMotorStatus, rqt_gui_cpp::Plugin)
