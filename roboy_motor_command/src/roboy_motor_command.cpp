#include <roboy_motor_command/roboy_motor_command.hpp>

RoboyMotorCommand::RoboyMotorCommand()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorCommand");
}

void RoboyMotorCommand::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    QScrollArea* scrollArea = widget_->findChild<QScrollArea *>("motor_command");
    scrollArea->setBackgroundRole(QPalette::Window);
    scrollArea->setFrameShadow(QFrame::Plain);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);

    //vertical box that contains all the checkboxes for the filters
    motor_command_scrollarea = new QWidget(widget_);
    motor_command_scrollarea->setObjectName("motor_command_scrollarea");
    motor_command_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    motor_command_scrollarea->setLayout(new QVBoxLayout(motor_command_scrollarea));
    scrollArea->setWidget(motor_command_scrollarea);

    addFpgaWidgets(ui.fpga->value());

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_command_rqt_plugin");
    }

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    if(nh->hasParam("number_of_fpgas")){
        nh->getParam("number_of_fpgas",number_of_fpgas);
        ROS_INFO("found number_of_fpgas %d on parameter server", number_of_fpgas);
    }

    setpoint_slider_widget_all = widget_->findChild<QSlider *>("motor_setPoint_slider_all");
    QObject::connect(setpoint_slider_widget_all, SIGNAL(valueChanged(int)), this, SLOT(setPointAllChangedSlider()));
    setpoint_widget_all = widget_->findChild<QLineEdit *>("motor_setPoint_all");
    QObject::connect(setpoint_widget_all, SIGNAL(editingFinished()), this, SLOT(setPointAllChanged()));
    scale = widget_->findChild<QLineEdit *>("motor_scale");

    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    for(auto fpga:fpga_names) {
        motorControl[fpga_id_from_name[fpga]] = nh->serviceClient<roboy_middleware_msgs::ControlMode>(
                "/roboy/"+fpga+"/middleware/ControlMode");
        motorConfig[fpga_id_from_name[fpga]] = nh->serviceClient<roboy_middleware_msgs::MotorConfigService>(
                "/roboy/"+fpga+"/middleware/MotorConfig");
        emergencyStop[fpga_id_from_name[fpga]] = nh->serviceClient<std_srvs::SetBool>("/roboy/"+fpga+"/middleware/EmergencyStop");
    }

    ui.stop_button_all->setStyleSheet("background-color: green");
    QObject::connect(ui.stop_button_all, SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));

    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.force, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.direct, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(addFpgaWidgets(int)));

    QObject::connect(ui.update_config, SIGNAL(clicked()), this, SLOT(update_config()));

    QObject::connect(ui.load_motor_config, SIGNAL(clicked()), this, SLOT(loadMotorConfig()));
    loadMotorConfig();
}

void RoboyMotorCommand::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyMotorCommand::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorCommand::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorCommand::stopButtonAllClicked(){
    std_srvs::SetBool msg;
    if(ui.stop_button_all->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        for(uint fpga=2;fpga<=6;fpga++)
            emergencyStop[fpga].call(msg);
        ui.motor_command->setEnabled(false);
        ui.pos->setEnabled(false);
        ui.vel->setEnabled(false);
        ui.dis->setEnabled(false);
        ui.force->setEnabled(false);
        ui.direct->setEnabled(false);
    }else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        for(uint fpga=2;fpga<=6;fpga++)
            emergencyStop[fpga].call(msg);
        ui.motor_command->setEnabled(true);
        ui.pos->setEnabled(true);
        ui.vel->setEnabled(true);
        ui.dis->setEnabled(true);
        ui.force->setEnabled(true);
        ui.direct->setEnabled(true);
    }
}

void RoboyMotorCommand::setPointChanged(){
    roboy_middleware_msgs::MotorCommand msg;
    bool ok;
    double motor_scale = scale->text().toDouble(&ok);
    if(!ok)
        return;
    int fpga = ui.fpga->value();
    for (uint motor = 0; motor < total_number_of_motors; motor++) {
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
        if (it == active_motors[fpga].end())
            continue;
        double setPoint = setpoint_widget[motor]->text().toDouble(&ok) * motor_scale;
        if(setpoint[motor] != setPoint && ok) {
            setpoint[motor] = setPoint;
            msg.id = ui.fpga->value();
            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
            if (control_mode[motor] == FORCE) {
                double displacement = force2displacement(setPoint, msg.id, motor % NUMBER_OF_MOTORS_PER_FPGA);
                msg.set_points.push_back(displacement);
            } else {
                msg.set_points.push_back(setPoint);
            }
        }
        setpoint_widget[motor]->setText(QString::number(setPoint));
    }
    motorCommand.publish(msg);
}

void RoboyMotorCommand::setPointChangedSlider(){
    roboy_middleware_msgs::MotorCommand msg;
    bool ok;
    double motor_scale = scale->text().toDouble(&ok);
    if(!ok){
        ROS_ERROR("invalid scale");
        return;
    }
    int fpga = ui.fpga->value();
    for (uint motor = 0; motor < total_number_of_motors; motor++) {
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
        if (it == active_motors[fpga].end())
            continue;
        double setPoint = (setpoint_slider_widget[motor]->value()-50.0) * motor_scale;
        if(setpoint[motor] != setPoint ) {
            setpoint[motor] = setPoint;
            msg.id = ui.fpga->value();
            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
            if (control_mode[motor] == FORCE) {
                double displacement = force2displacement(setPoint, msg.id, motor % NUMBER_OF_MOTORS_PER_FPGA);
                msg.set_points.push_back(displacement);
            } else {
                msg.set_points.push_back(setPoint);
            }
        }
        setpoint_widget[motor]->setText(QString::number(setPoint));
    }
    motorCommand.publish(msg);
}

void RoboyMotorCommand::setPointAllChanged(){
    roboy_middleware_msgs::MotorCommand msg;
    bool ok;
    double motor_scale = scale->text().toDouble(&ok);
    if(!ok){
        ROS_ERROR("invalid scale");
        return;
    }
    int fpga = ui.fpga->value();
    double setPoint = setpoint_widget_all->text().toDouble(&ok) * motor_scale;
    for (uint motor = 0; motor < total_number_of_motors; motor++) {
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
        if (it == active_motors[fpga].end())
            continue;
        if(setpoint[motor] != setPoint) {
            setpoint[motor] = setPoint;
            msg.id = ui.fpga->value();
            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
            if (control_mode[motor] == FORCE) {
                double displacement = force2displacement(setPoint, msg.id, motor % NUMBER_OF_MOTORS_PER_FPGA);
                msg.set_points.push_back(displacement);
            } else {
                msg.set_points.push_back(setPoint);
            }
        }
        setpoint_widget[motor]->setText(QString::number(setPoint));
    }
    motorCommand.publish(msg);
}

void RoboyMotorCommand::setPointAllChangedSlider(){
    roboy_middleware_msgs::MotorCommand msg;
    bool ok;
    double motor_scale = scale->text().toDouble(&ok);
    if(!ok){
        ROS_ERROR("invalid scale");
        return;
    }
    int fpga = ui.fpga->value();
    double setPoint = (setpoint_slider_widget_all->value()-50.0) * motor_scale;
    ui.motor_setPoint_all->setText(QString::number(setPoint));
    for (uint motor = 0; motor < total_number_of_motors; motor++) {
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
        if (it == active_motors[fpga].end())
            continue;
        if(setpoint[motor] != setPoint) {
            setpoint[motor] = setPoint;
            msg.id = ui.fpga->value();
            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
            if (control_mode[motor] == FORCE) {
                double displacement = force2displacement(setPoint, msg.id, motor % NUMBER_OF_MOTORS_PER_FPGA);
                msg.set_points.push_back(displacement);
            } else {
                msg.set_points.push_back(setPoint);
            }
        }
        setpoint_widget[motor]->setText(QString::number(setPoint));
    }
    motorCommand.publish(msg);
}

void RoboyMotorCommand::controlModeChanged(){
    roboy_middleware_msgs::ControlMode msg;
    int fpag = ui.fpga->value();
    if(ui.pos->isChecked()) {
        for(int motor = 0; motor<total_number_of_motors; motor++){
            vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
            if (it == active_motors[fpag].end())
                continue;
            control_mode[motor] = POSITION;
            pos[motor]->setChecked(true);
        }
        msg.request.control_mode = POSITION;
        ROS_INFO("changed to POSITION control");
    }
    if(ui.vel->isChecked()) {
        for(int motor = 0; motor<total_number_of_motors; motor++){
            vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
            if (it == active_motors[fpag].end())
                continue;
            control_mode[motor] = VELOCITY;
            vel[motor]->setChecked(true);
        }
        msg.request.control_mode = VELOCITY;
        ROS_INFO("changed to VELOCITY control");
    }
    if(ui.dis->isChecked()) {
        for(int motor = 0; motor<total_number_of_motors; motor++){
            vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
            if (it == active_motors[fpag].end())
                continue;
            control_mode[motor] = DISPLACEMENT;
            dis[motor]->setChecked(true);
        }
        msg.request.control_mode = DISPLACEMENT;
        ROS_INFO("changed to DISPLACEMENT control");
    }
    if(ui.force->isChecked()) {
        for(int motor = 0; motor<total_number_of_motors; motor++){
            vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
            if (it == active_motors[fpag].end())
                continue;
            control_mode[motor] = FORCE;
            force[motor]->setChecked(true);
        }
        msg.request.control_mode = DISPLACEMENT;
        ROS_INFO("changed to FORCE control");
    }
    if(ui.direct->isChecked()) {
        for(int motor = 0; motor<total_number_of_motors; motor++){
            vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
            if (it == active_motors[fpag].end())
                continue;
            control_mode[motor] = DIRECT_PWM;
            dir[motor]->setChecked(true);
        }
        msg.request.control_mode = DIRECT_PWM;
        ROS_INFO("changed to DIRECT_PWM control");
    }

    bool ok;
    double motor_scale = scale->text().toDouble(&ok);
    if(!ok){
        ROS_ERROR("invalid scale");
        return;
    }
    msg.request.set_point = 0;
    if(!motorControl[ui.fpga->value()].call(msg))
        ROS_ERROR("failed to change control mode of head, is emergency stop active?! are the fpgas connected?!");
}

void RoboyMotorCommand::update_config(){
    bool ok;
    int outputPosMax = ui.outputPosMax->text().toInt(&ok);
    if(!ok || outputPosMax>4000) {
        ROS_ERROR("outputPosMax not valid [0 to 4000]");
        return;
    }
    int outputNegMax = ui.outputNegMax->text().toInt(&ok);
    if(!ok || outputNegMax<-4000) {
        ROS_ERROR("outputNegMax not valid [0 to -4000]");
        return;
    }
    int spPosMax = ui.spNegMax->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("spPosMax not valid");
        return;
    }
    int spNegMax = ui.spNegMax->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("spNegMax not valid");
        return;
    }
    int integralPosMax = ui.integralPosMax->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("integralPosMax not valid");
        return;
    }
    int integralNegMax = ui.integralNegMax->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("integralNegMax not valid");
        return;
    }
    int deadband = ui.deadband->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("deadband not valid");
        return;
    }
    int Kp = ui.Kp->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("Kp not valid");
        return;
    }
    int Ki = ui.Ki->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("Ki not valid");
        return;
    }
    int Kd = ui.Kd->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("Kd not valid");
        return;
    }
    int forwardGain = ui.forwardGain->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("forwardGain not valid");
        return;
    }
    int outputDivider = ui.outputDivider->text().toInt(&ok);
    if(!ok) {
        ROS_ERROR("outputDivider not valid");
        return;
    }
    roboy_middleware_msgs::MotorConfigService msg;
    int motor_config_control_mode= 0;
    if(ui.pos_motor_config->isChecked())
        motor_config_control_mode = POSITION;
    if(ui.vel_motor_config->isChecked())
        motor_config_control_mode = VELOCITY;
    if(ui.dis_motor_config->isChecked())
        motor_config_control_mode = DISPLACEMENT;
    if(ui.cur_motor_config->isChecked())
        motor_config_control_mode = CURRENT;
    if(ui.dir_motor_config->isChecked())
        motor_config_control_mode = DIRECT_PWM;
    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++){
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), i);
        if (it == active_motors[ui.fpga->value()].end())
            continue;
        msg.request.config.motors.push_back(i);
        msg.request.config.setpoint.push_back(0);
        msg.request.config.control_mode.push_back(motor_config_control_mode);
        msg.request.config.output_pos_max.push_back(outputPosMax);
        msg.request.config.output_neg_max.push_back(outputNegMax);
        msg.request.config.sp_pos_max.push_back(spPosMax);
        msg.request.config.sp_neg_max.push_back(spNegMax);
        msg.request.config.integral_pos_max.push_back(integralPosMax);
        msg.request.config.integral_neg_max.push_back(integralNegMax);
        msg.request.config.dead_band.push_back(deadband);
        msg.request.config.kp.push_back(Kp);
        msg.request.config.ki.push_back(Ki);
        msg.request.config.kd.push_back(Kd);
        msg.request.config.forward_gain.push_back(forwardGain);
        msg.request.config.output_divider.push_back(outputDivider);
    }
    motorConfig[ui.fpga->value()].call(msg);
}

void RoboyMotorCommand::loadMotorConfig(){
    readConfig(ui.motor_config_path->text().toStdString());
}

void RoboyMotorCommand::addFpgaWidgets(int fpga){
    for(auto w:widgets) {
        motor_command_scrollarea->layout()->removeWidget(w);
        delete w;
    }
    widgets.clear();
    setpoint_widget.clear();
    setpoint_slider_widget.clear();
    pos.clear();
    vel.clear();
    dis.clear();
    force.clear();
    cur.clear();
    dir.clear();

    ui.fpga_name->setText(fpga_name_from_id[fpga].c_str());

    total_number_of_motors = NUMBER_OF_MOTORS_PER_FPGA;
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        vector<int>::iterator it = find (active_motors[ui.fpga->value()].begin(), active_motors[ui.fpga->value()].end(), motor);
        if (it == active_motors[fpga].end())
            continue;
        QWidget *widget = new QWidget(motor_command_scrollarea);
        char str[100];
        sprintf(str, "motor%d_%d", fpga, motor);
        widget->setObjectName(str);
        widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        widget->setLayout(new QHBoxLayout(widget));

        QLabel *label = new QLabel(widget);
        sprintf(str, "%d", motor);
        label->setFixedSize(20,30);
        label->setText(str);
        widget->layout()->addWidget(label);

        bool active = true;

        QRadioButton *p = new QRadioButton(widget);
        p->setText("pos");
        p->setFixedSize(50,30);
        p->setCheckable(true);
        p->setObjectName("pos");
        pos[motor]= p;
        QObject::connect(p, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        widget->layout()->addWidget(p);

        QRadioButton *v = new QRadioButton(widget);
        v->setText("vel");
        v->setFixedSize(50,30);
        v->setCheckable(true);
        v->setObjectName("vel");
        widget->layout()->addWidget(v);
        vel[motor]= v;
        QObject::connect(v, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *d = new QRadioButton(widget);
        d->setText("dis");
        d->setFixedSize(50,30);
        d->setCheckable(true);
        d->setObjectName("dis");
        d->setChecked(true);
        setpoint.push_back(0);
        control_mode.push_back(DISPLACEMENT);
        widget->layout()->addWidget(d);
        dis[motor]= d;
        QObject::connect(d, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *f = new QRadioButton(widget);
        f->setText("force");
        f->setFixedSize(50,30);
        f->setCheckable(true);
        f->setObjectName("force");
        widget->layout()->addWidget(f);
        force[motor] = f;
        QObject::connect(f, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *di = new QRadioButton(widget);
        di->setText("pwm");
        di->setFixedSize(50,30);
        di->setCheckable(true);
        di->setObjectName("pwm");
        di->setChecked(true);
        setpoint.push_back(0);
        control_mode.push_back(DIRECT_PWM);
        widget->layout()->addWidget(di);
        dir[motor]=di;
        QObject::connect(di, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QLineEdit *line = new QLineEdit(widget);
        line->setFixedSize(70,30);
        if(!active)
            line->setEnabled(false);
        widget->layout()->addWidget(line);
        setpoint_widget[motor] = line;
        QObject::connect(line, SIGNAL(editingFinished()), this, SLOT(setPointChanged()));
        setpoint_widget[motor]->setText(QString::number(setpoint[motor]));

        QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
        slider->setFixedSize(100,30);
        slider->setValue(50);
        if(!active)
            slider->setEnabled(false);
        widget->layout()->addWidget(slider);
        setpoint_slider_widget[motor] = slider;

        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setPointChangedSlider()));

        motor_command_scrollarea->layout()->addWidget(widget);

        widgets.push_back(widget);
    }
}

PLUGINLIB_EXPORT_CLASS(RoboyMotorCommand, rqt_gui_cpp::Plugin)
