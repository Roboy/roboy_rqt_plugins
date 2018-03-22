#include <roboy_control/roboy_control.hpp>

RoboyControl::RoboyControl()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyControl");
}

void RoboyControl::initPlugin(qt_gui_cpp::PluginContext &context) {
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
        ros::init(argc, argv, "motor_control_rqt_plugin");
    }

    motor["head"].push_back(ui.head_motor_0);
    motor["head"].push_back(ui.head_motor_1);
    motor["head"].push_back(ui.head_motor_2);
    motor["head"].push_back(ui.head_motor_3);

    motor["arm_left"].push_back(ui.arm_left_motor_0);
    motor["arm_left"].push_back(ui.arm_left_motor_1);
    motor["arm_left"].push_back(ui.arm_left_motor_2);
    motor["arm_left"].push_back(ui.arm_left_motor_3);
    motor["arm_left"].push_back(ui.arm_left_motor_4);
    motor["arm_left"].push_back(ui.arm_left_motor_5);
    motor["arm_left"].push_back(ui.arm_left_motor_6);
    motor["arm_left"].push_back(ui.arm_left_motor_7);
    motor["arm_left"].push_back(ui.arm_left_motor_8);
    motor["arm_left"].push_back(ui.arm_left_motor_9);
    motor["arm_left"].push_back(ui.arm_left_motor_10);

    motor["arm_right"].push_back(ui.arm_right_motor_0);
    motor["arm_right"].push_back(ui.arm_right_motor_1);
    motor["arm_right"].push_back(ui.arm_right_motor_2);
    motor["arm_right"].push_back(ui.arm_right_motor_3);
    motor["arm_right"].push_back(ui.arm_right_motor_4);
    motor["arm_right"].push_back(ui.arm_right_motor_5);
    motor["arm_right"].push_back(ui.arm_right_motor_6);
    motor["arm_right"].push_back(ui.arm_right_motor_7);
    motor["arm_right"].push_back(ui.arm_right_motor_8);
    motor["arm_right"].push_back(ui.arm_right_motor_9);
    motor["arm_right"].push_back(ui.arm_right_motor_10);

    motor["leg_left"].push_back(ui.leg_left_motor_0);
    motor["leg_left"].push_back(ui.leg_left_motor_1);
    motor["leg_left"].push_back(ui.leg_left_motor_2);
    motor["leg_left"].push_back(ui.leg_left_motor_3);
    motor["leg_left"].push_back(ui.leg_left_motor_4);
    motor["leg_left"].push_back(ui.leg_left_motor_5);
    motor["leg_left"].push_back(ui.leg_left_motor_6);
    motor["leg_left"].push_back(ui.leg_left_motor_7);
    motor["leg_left"].push_back(ui.leg_left_motor_8);
    motor["leg_left"].push_back(ui.leg_left_motor_9);

    motor["leg_right"].push_back(ui.leg_right_motor_0);
    motor["leg_right"].push_back(ui.leg_right_motor_1);
    motor["leg_right"].push_back(ui.leg_right_motor_2);
    motor["leg_right"].push_back(ui.leg_right_motor_3);
    motor["leg_right"].push_back(ui.leg_right_motor_4);
    motor["leg_right"].push_back(ui.leg_right_motor_5);
    motor["leg_right"].push_back(ui.leg_right_motor_6);
    motor["leg_right"].push_back(ui.leg_right_motor_7);
    motor["leg_right"].push_back(ui.leg_right_motor_8);
    motor["leg_right"].push_back(ui.leg_right_motor_9);

    for(auto &body_part:motor){
        for(auto &m:body_part.second)
            m->setStyleSheet("background-color: red");
    }

    ui.stop_button->setStyleSheet("background-color: green");
    QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));

//    for(uint motor = 0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++){
//        QObject::connect(setpoint_slider_widget.at(motor), SIGNAL(valueChanged(int)), this, SLOT(setPointChanged(int)));
//    }
//    QObject::connect(setpoint_slider_widget.at(NUMBER_OF_MOTORS_PER_FPGA), SIGNAL(valueChanged(int)), this, SLOT(setPointAllChanged(int)));

//    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.force, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//    QObject::connect(ui.update_config, SIGNAL(clicked()), this, SLOT(update_config()));
//
//    QObject::connect(ui.load_motor_config, SIGNAL(clicked()), this, SLOT(loadMotorConfig()));
}

void RoboyControl::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyControl::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyControl::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyControl::stopButtonClicked(){
    std_srvs::SetBool msg;
    if(ui.stop_button->isChecked()) {
        ui.stop_button->setStyleSheet("background-color: red");
        msg.request.data = 1;
//        emergencyStop.call(msg);
    }else {
        ui.stop_button->setStyleSheet("background-color: green");
        msg.request.data = 0;
//        emergencyStop.call(msg);
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_control, RoboyControl, RoboyControl, rqt_gui_cpp::Plugin)
