#include <vr_puppets/vr_puppets.hpp>
#include <QtWidgets/QCheckBox>

VRPuppets::VRPuppets()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("VRPuppets");
}

void VRPuppets::initPlugin(qt_gui_cpp::PluginContext &context) {
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
    motor_status = nh->advertise<roboy_middleware_msgs::MotorStatus>("/VRpuppet/MotorStatus",1);
    motor_command_sub = nh->subscribe("/VRpuppet/MotorCommand",1,&VRPuppets::MotorCommand, this);

    zero_srv = nh->serviceClient<std_srvs::Empty>("/stepper_motor_shield/zero");
    e_stop_server = nh->advertiseService("/m3/emergency_stop",&VRPuppets::EmergencyCallback,this);
    control_mode_srv = nh->advertiseService("/VRpuppet/ControlMode",&VRPuppets::ControlMode,this);

    QObject::connect(this, SIGNAL(new_data()), this, SLOT(plotData()));
    QObject::connect(this, SIGNAL(new_motor()), this, SLOT(newMotor()));
    QObject::connect(ui.all_to_position, SIGNAL(clicked()), this, SLOT(allToPosition()));
    QObject::connect(ui.all_to_velocity, SIGNAL(clicked()), this, SLOT(allToVelocity()));
    QObject::connect(ui.all_to_displacement, SIGNAL(clicked()), this, SLOT(allToDisplacement()));
    QObject::connect(ui.stop, SIGNAL(clicked()), this, SLOT(stop()));
    QObject::connect(ui.motor0, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor1, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor2, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor3, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor4, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.motor5, SIGNAL(valueChanged(int)), this, SLOT(sendMotorCommandLinearActuators()));
    QObject::connect(ui.serial_node, SIGNAL(clicked()), this, SLOT(serialNode()));
    QObject::connect(ui.zero, SIGNAL(clicked()), this, SLOT(zero()));
    ui.stop->setStyleSheet("background-color: red");
    QObject::connect(ui.setpoint_all, SIGNAL(valueChanged(int)), this, SLOT(sliderMovedAll()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

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

    uint32_t ip;
    inet_pton(AF_INET, "192.168.255.255", &ip);
    udp.reset(new UDPSocket(8000));
    udp_command.reset(new UDPSocket(8001));
    udp_thread.reset(new std::thread(&VRPuppets::receiveStatusUDP, this));
    udp_thread->detach();

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

    updateMotorCommands();

    initialized = true;
}

void VRPuppets::shutdownPlugin() {
    motorStatus.shutdown();
}

void VRPuppets::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void VRPuppets::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void VRPuppets::receiveStatusUDP() {
    ROS_INFO("start receiving udp");
    ros::Time t0 = ros::Time::now(), t1;
    while (ros::ok()) {
        int bytes_received = udp->receiveUDPFromClient();
        if (bytes_received == 20) {
            t1 = ros::Time::now();
            ros::Duration d = (t1-t0);
            t0 = t1;
            float hz = d.toSec();
            int approx_hz = hz;
            ROS_INFO_THROTTLE(60,"receiving motor status at %f Hz", hz);
            ros::Duration delta = (ros::Time::now() - start_time);
            lock_guard<mutex> lock(mux);
            int motor = udp->buf[0];
            auto it = ip_address.find(motor);
            if (it == ip_address.end()) {;
                char IP[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &udp->client_addr.sin_addr, IP, INET_ADDRSTRLEN);
                ROS_INFO("new motor %d %s", motor, IP);
                ip_address[motor] = IP;
                Q_EMIT new_motor();
                break;
            }
            time.push_back(delta.toSec());
            int32_t pos = (int32_t) ((uint8_t) udp->buf[7] << 24 | (uint8_t) udp->buf[6] << 16 |
                                        (uint8_t) udp->buf[5] << 8 | (uint8_t) udp->buf[4]);
            int32_t vel = (int32_t) ((uint8_t) udp->buf[11] << 24 | (uint8_t) udp->buf[10] << 16 |
                                     (uint8_t) udp->buf[9] << 8 | (uint8_t) udp->buf[8]);
            int32_t dis = (int32_t) ((uint8_t) udp->buf[15] << 24 | (uint8_t) udp->buf[14] << 16 |
                                     (uint8_t) udp->buf[13] << 8 | (uint8_t) udp->buf[12]);
            int32_t pwm = (int32_t) ((uint8_t) udp->buf[19] << 24 | (uint8_t) udp->buf[18] << 16 |
                                     (uint8_t) udp->buf[17] << 8 | (uint8_t) udp->buf[16]);
//            ROS_INFO_THROTTLE(1,"%d",vel);
            motor_position[motor].push_back(pos);
            motor_velocity[motor].push_back(vel);
            motor_displacement[motor].push_back(dis);
            motor_pwm[motor].push_back(pwm);
            for(auto m:ip_address){
                if(m.first==motor)
                    continue;
                motor_position[m.first].push_back(motor_position[m.first].back());
                motor_velocity[m.first].push_back(motor_velocity[m.first].back());
                motor_displacement[m.first].push_back(motor_displacement[m.first].back());
                motor_pwm[m.first].push_back(motor_pwm[m.first].back());
            }
//            ROS_INFO_THROTTLE(1,"receiving status from motor %d, pos=%d, vel=%d, dis=%d, pwm=%d",motor,pos,vel,dis,pwm);
            if (motor_position[motor].size() > samples_per_plot) {
                for(auto m:ip_address) {
                    motor_position[m.first].pop_front();
                    motor_velocity[m.first].pop_front();
                    motor_displacement[m.first].pop_front();
                    motor_pwm[m.first].pop_front();
                }
            }
            if (time.size() > samples_per_plot)
                time.pop_front();

            if ((counter++) % (100) == 0) {
                Q_EMIT new_data();
                roboy_middleware_msgs::MotorStatus msg;
                msg.id = 69;
                msg.power_sense = !ui.stop->isChecked();
                for(auto m:ip_address){
                    msg.position.push_back(motor_position[m.first].back());
                    msg.velocity.push_back(motor_velocity[m.first].back());
                    msg.displacement.push_back(motor_displacement[m.first].back());
                    msg.pwm_ref.push_back(motor_pwm[m.first].back());
                }
                motor_status.publish(msg);
            }
            if ((counter % 500) == 0 && initialized) {
                rescale();
            }
        }
    }
    ROS_INFO("stop receiving udp");
}

void VRPuppets::updateMotorCommands(){
    for(auto w:widgets) {
        motor_command_scrollarea->layout()->removeWidget(w);
        delete w;
    }
    widgets.clear();

    for (auto m:ip_address) {
        QWidget *widget = new QWidget(motor_command_scrollarea);
        widget->setObjectName(m.second.c_str());
        widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        widget->setLayout(new QHBoxLayout(widget));

        QCheckBox *c = new QCheckBox(widget);
        c->setFixedSize(20,30);
        c->setCheckable(true);
        c->setChecked(true);
        check[m.first] = c;
        widget->layout()->addWidget(c);

        QLabel *label2 = new QLabel(widget);
        label2->setFixedSize(30,30);
        char str[22];
        sprintf(str, "ID %d", m.first);
        label2->setText(str);
        widget->layout()->addWidget(label2);

        QLabel *label = new QLabel(widget);
        label->setFixedSize(100,30);
        label->setText(m.second.c_str());
        widget->layout()->addWidget(label);

        QRadioButton *p = new QRadioButton(widget);
        p->setText("pos");
        p->setFixedSize(50,30);
        p->setCheckable(true);
        p->setObjectName("pos");
        pos[m.first] = p;
        QObject::connect(p, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        widget->layout()->addWidget(p);

        QRadioButton *v = new QRadioButton(widget);
        v->setText("vel");
        v->setFixedSize(50,30);
        v->setCheckable(true);
        v->setObjectName("vel");
        widget->layout()->addWidget(v);
        vel[m.first] = v;
        QObject::connect(v, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QRadioButton *d = new QRadioButton(widget);
        d->setText("dis");
        d->setFixedSize(50,30);
        d->setCheckable(true);
        d->setObjectName("dis");
        d->setChecked(true);
        control_mode[m.first] = DISPLACEMENT;
        widget->layout()->addWidget(d);
        dis[m.first] = d;
        QObject::connect(d, SIGNAL(clicked()), this, SLOT(controlModeChanged()));

        QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
        slider->setFixedSize(100,30);
        slider->setValue(50);
        widget->layout()->addWidget(slider);
        sliders[m.first] = slider;

        QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(sliderMoved()));

        motor_command_scrollarea->layout()->addWidget(widget);
        widgets.push_back(widget);
    }
}

void VRPuppets::MotorCommand( const roboy_middleware_msgs::MotorCommand::ConstPtr &msg){
    if(msg->id==69){
        for(int i=0;i<msg->motors.size();i++){
            set_points[msg->motors[i]] = msg->set_points[i];
        }
        sendCommand();
    }
}

void VRPuppets::plotData() {
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

void VRPuppets::rescale(){
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

void VRPuppets::sendCommand(){
        udp_command->client_addr.sin_port = htons(8001);
        udp_command->numbytes = 10;
        for (auto m:ip_address) {
            mempcpy(udp_command->buf, &set_points[m.first], 4);
            mempcpy(&udp_command->buf[4], &m.first, 4);
            udp_command->numbytes = 10;
            udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
            udp_command->sendUDPToClient();
        }
}

void VRPuppets::controlModeChanged(){
    lock_guard<mutex> lock(mux);

    int Kp, Ki, Kd;
    for(auto m:ip_address) {
        bool ok;
        if (control_mode[m.first]==POSITION){
            Kp = ui.Kp_pos->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_pos->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_pos->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        }else if(control_mode[m.first]==VELOCITY) {
            Kp = ui.Kp_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_vel->text().toInt(&ok);
            if (!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        }else if(control_mode[m.first]==DISPLACEMENT) {
            Kp = ui.Kp_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Kp");
                return;
            }
            Ki = ui.Ki_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Ki");
                return;
            }
            Kd = ui.Kd_dis->text().toInt(&ok);
            if(!ok) {
                ROS_ERROR("invalid conversion to integer of Kd");
                return;
            }
        }else{
            ROS_ERROR("haeeee????");
            return;
        }
        mempcpy(&udp_command->buf[0],&Kd,4);
        mempcpy(&udp_command->buf[4],&Ki,4);
        mempcpy(&udp_command->buf[8],&Kp,4);
        mempcpy(&udp_command->buf[12],&control_mode[m.first],4);
        mempcpy(&udp_command->buf[16],&m.first,4);

        udp_command->client_addr.sin_port = htons(8001);
        udp_command->numbytes = 20;
        udp_command->client_addr.sin_addr.s_addr = inet_addr(m.second.c_str());
        udp_command->sendUDPToClient();

        if(control_mode[m.first]) {
            udp_command->numbytes = 10;
            set_points[m.first] = motor_position[m.first].back();
            ui.setpoint->setText(ui.setpoint_pos->text());
            mempcpy(udp_command->buf, &set_points[m.first], 4);
            mempcpy(&udp_command->buf[4], &m.first, 4);
        }
    }
}

void VRPuppets::allToPosition(){
    bool ok;
    for(auto m:ip_address){
        control_mode[m.first] = POSITION;
        pos[m.first]->setChecked(true);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(false);
        set_points[m.first] = ui.setpoint_pos->text().toInt(&ok);
    }
    ui.setpoint->setText(ui.setpoint_pos->text());
    controlModeChanged();
    sendCommand();
}

void VRPuppets::allToVelocity(){
    bool ok;
    for(auto m:ip_address){
        control_mode[m.first] = VELOCITY;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(true);
        dis[m.first]->setChecked(false);
        set_points[m.first] = ui.setpoint_vel->text().toInt(&ok);
    }
    ui.setpoint->setText(ui.setpoint_vel->text());
    controlModeChanged();
    sendCommand();
}

void VRPuppets::allToDisplacement(){
    bool ok;
    for(auto m:ip_address){
        control_mode[m.first] = DISPLACEMENT;
        pos[m.first]->setChecked(false);
        vel[m.first]->setChecked(false);
        dis[m.first]->setChecked(true);
        set_points[m.first] = ui.setpoint_dis->text().toInt(&ok);
    }
    ui.setpoint->setText(ui.setpoint_dis->text());
    controlModeChanged();
    sendCommand();
}

void VRPuppets::sliderMoved(){
    bool ok;
    for(auto m:ip_address) {
        if(check[m.first]->isChecked()) {
            int motor_scale = ui.scale->text().toInt(&ok);
            if (!ok){
                ROS_ERROR("motor scale invalid");
                return;
            }
            set_points[m.first] = (sliders[m.first]->value() - 50) * motor_scale;
        }
    }
    sendCommand();
}

void VRPuppets::sliderMovedAll(){
    bool ok;
    int motor_scale = ui.scale->text().toInt(&ok);
    for(auto m:ip_address) {
        if(check[m.first]->isChecked()) {
            if (!ok) {
                ROS_ERROR("motor scale invalid");
                return;
            }
            set_points[m.first] = (ui.setpoint_all->value() - 50) * motor_scale;
        }else{
            ROS_WARN("ignoring motor %d because it is not activated", m.first);
        }
    }
    char str[100];
    sprintf(str,"%d",(ui.setpoint_all->value() - 50) * motor_scale);
    ui.setpoint->setText(str);
    sendCommand();
}

void VRPuppets::stop(){
    if(!ui.stop->isChecked()) {
        ui.stop->setStyleSheet("background-color: red");
        bool ok;
        char str[100];
        for (auto m:ip_address) {
            switch (control_mode[m.first]) {
                case POSITION:
                    sprintf(str,"%d", Kp[m.first]);
                    ui.Kp_pos->setText(str);
                    sprintf(str,"%d", Ki[m.first]);
                    ui.Ki_pos->setText(str);
                    sprintf(str,"%d", Kd[m.first]);
                    ui.Kd_pos->setText(str);
                    break;
                case VELOCITY:
                    sprintf(str,"%d", Kp[m.first]);
                    ui.Kp_vel->setText(str);
                    sprintf(str,"%d", Ki[m.first]);
                    ui.Ki_vel->setText(str);
                    sprintf(str,"%d", Kd[m.first]);
                    ui.Kd_vel->setText(str);
                    break;
                case DISPLACEMENT:
                    sprintf(str,"%d", Kp[m.first]);
                    ui.Kp_dis->setText(str);
                    sprintf(str,"%d", Ki[m.first]);
                    ui.Ki_dis->setText(str);
                    sprintf(str,"%d", Kd[m.first]);
                    ui.Kd_dis->setText(str);
                    break;
            }

            ui.pos_frame->setEnabled(true);
            ui.vel_frame->setEnabled(true);
            ui.dis_frame->setEnabled(true);
            ui.motor_command->setEnabled(true);
            ui.setpoint_all->setEnabled(true);
            ui.stop->setText("STOP");
        }
        controlModeChanged();
        sendCommand();
    }else{
        ui.stop->setStyleSheet("background-color: green");
        bool ok;
        for (auto m:ip_address) {
            switch (control_mode[m.first]) {
                case POSITION:
                    Kp[m.first] = ui.Kp_pos->text().toInt(&ok);
                    Ki[m.first] = ui.Ki_pos->text().toInt(&ok);
                    Kd[m.first] = ui.Kd_pos->text().toInt(&ok);
                    ui.Kp_pos->setText("0");
                    ui.Ki_pos->setText("0");
                    ui.Kd_pos->setText("0");
                    break;
                case VELOCITY:
                    Kp[m.first] = ui.Kp_vel->text().toInt(&ok);
                    Ki[m.first] = ui.Ki_vel->text().toInt(&ok);
                    Kd[m.first] = ui.Kd_vel->text().toInt(&ok);
                    ui.Kp_vel->setText("0");
                    ui.Ki_vel->setText("0");
                    ui.Kd_vel->setText("0");
                    break;
                case DISPLACEMENT:
                    Kp[m.first] = ui.Kp_dis->text().toInt(&ok);
                    Ki[m.first] = ui.Ki_dis->text().toInt(&ok);
                    Kd[m.first] = ui.Kd_dis->text().toInt(&ok);
                    ui.Kp_dis->setText("0");
                    ui.Ki_dis->setText("0");
                    ui.Kd_dis->setText("0");
                    break;
            }
            ui.pos_frame->setEnabled(false);
            ui.vel_frame->setEnabled(false);
            ui.dis_frame->setEnabled(false);
            ui.motor_command->setEnabled(false);
            ui.setpoint_all->setEnabled(false);
            ui.stop->setText("CONTINUE");
        }
        controlModeChanged();
    }
}

void VRPuppets::sendMotorCommandLinearActuators(){
    roboy_middleware_msgs::MotorCommand msg;
    msg.id = 68;
    msg.motors = {0,1,2,3,4,5,6,7,8,9};
    msg.set_points = {(float)ui.motor0->value(),
                      (float)ui.motor1->value(),
                      (float)ui.motor2->value(),
                      (float)ui.motor3->value(),
                      (float)ui.motor4->value(),
                      (float)ui.motor5->value(),0,0,0,0};
    motor_command.publish(msg);
}

void VRPuppets::serialNode(){
    system("rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0&");
}

bool VRPuppets::EmergencyCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
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

bool VRPuppets::ControlMode(roboy_middleware_msgs::ControlMode::Request &req, roboy_middleware_msgs::ControlMode::Response &res){
    if(req.motor_id.empty()){
        for(auto m:ip_address){
            control_mode[m.first] = req.control_mode;
            switch(req.control_mode){
                case 0:
                    pos[m.first]->setChecked(true);
                    vel[m.first]->setChecked(false);
                    dis[m.first]->setChecked(false);
                    break;
                case 1:
                    pos[m.first]->setChecked(false);
                    vel[m.first]->setChecked(true);
                    dis[m.first]->setChecked(false);
                    break;
                case 2:
                    pos[m.first]->setChecked(false);
                    vel[m.first]->setChecked(false);
                    dis[m.first]->setChecked(true);
                    break;
            }
        }
        controlModeChanged();
        sendCommand();
        switch(req.control_mode){
            case 0:
                ROS_INFO("control mode changed for all muscles to POSITION");
                break;
            case 1:
                ROS_INFO("control mode changed for all muscles to VELOCITY");
                break;
            case 2:
                ROS_INFO("control mode changed for all muscles to DISPLACEMENT");
                break;
        }

    }else{
        for(auto m:req.motor_id){
            control_mode[m] = req.control_mode;
            set_points[m] = req.set_point;
            switch (req.control_mode) {
                case 0:
                    pos[m]->setChecked(true);
                    vel[m]->setChecked(false);
                    dis[m]->setChecked(false);
                    break;
                case 1:
                    pos[m]->setChecked(false);
                    vel[m]->setChecked(true);
                    dis[m]->setChecked(false);
                    break;
                case 2:
                    pos[m]->setChecked(false);
                    vel[m]->setChecked(false);
                    dis[m]->setChecked(true);
                    break;
            }
        }
        controlModeChanged();
        sendCommand();
        switch(req.control_mode){
            case 0:
                ROS_INFO("control mode changed to POSITION");
                break;
            case 1:
                ROS_INFO("control mode changed to VELOCITY");
                break;
            case 2:
                ROS_INFO("control mode changed to DISPLACEMENT");
                break;
        }
    }
    return true;
}

void VRPuppets::zero(){
    std_srvs::Empty msg;
    zero_srv.call(msg);
}

void VRPuppets::newMotor(){
    time.clear();
    for(auto m:ip_address) {
        motor_position[m.first].clear();
        motor_velocity[m.first].clear();
        motor_displacement[m.first].clear();
        motor_pwm[m.first].clear();
    }
    updateMotorCommands();
    udp_thread.reset(new std::thread(&VRPuppets::receiveStatusUDP, this));
    udp_thread->detach();
}

PLUGINLIB_EXPORT_CLASS(VRPuppets, rqt_gui_cpp::Plugin)
