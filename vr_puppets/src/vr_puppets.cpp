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

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor%16])); // %16 because we only have 16 colors :(
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.displacement_plot->addGraph();
        ui.displacement_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        ui.current_plot->addGraph();
        ui.current_plot->graph(motor)->setPen(QPen(color_pallette[motor%16]));
        char str[20];
        sprintf(str,"motor_%d",motor);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        QObject::connect(box, SIGNAL(stateChanged(int)), this, SLOT(plotMotorChanged()));
        plotMotor[motor] = true;
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

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &VRPuppets::MotorStatus, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.toggle_all, SIGNAL(clicked()), this, SLOT(toggleAll()));
    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(fpgaChanged(int)));


    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

    uint32_t ip;
    inet_pton(AF_INET, "192.168.255.255", &ip);
    udp.reset(new UDPSocket(8000));
    udp_thread.reset(new std::thread(&VRPuppets::receiveStatusUDP, this));
    udp_thread->detach();
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
    while (ros::ok()) {
        int bytes_received = udp->receiveUDP();
        if (bytes_received == 20) {
            ros::Duration delta = (ros::Time::now() - start_time);
            time.push_back(delta.toSec());
            int motor = udp->buf[0];
            int32_t pos = (int32_t) ((uint8_t) udp->buf[7] << 24 | (uint8_t) udp->buf[6] << 16 |
                                        (uint8_t) udp->buf[5] << 8 | (uint8_t) udp->buf[4]);
            int32_t vel = (int32_t) ((uint8_t) udp->buf[11] << 24 | (uint8_t) udp->buf[10] << 16 |
                                     (uint8_t) udp->buf[9] << 8 | (uint8_t) udp->buf[8]);
            int32_t dis = (int32_t) ((uint8_t) udp->buf[15] << 24 | (uint8_t) udp->buf[14] << 16 |
                                     (uint8_t) udp->buf[13] << 8 | (uint8_t) udp->buf[12]);
            int32_t pwm = (int32_t) ((uint8_t) udp->buf[19] << 24 | (uint8_t) udp->buf[18] << 16 |
                                     (uint8_t) udp->buf[17] << 8 | (uint8_t) udp->buf[16]);
            motorData[3][motor][0].push_back(pos);
            motorData[3][motor][1].push_back(vel);
            motorData[3][motor][2].push_back(dis);
            motorData[3][motor][3].push_back(pwm);
//            ROS_INFO_THROTTLE(1,"receiving status from motor %d, pos=%d, vel=%d, dis=%d, pwm=%d",motor,pos,vel,dis,pwm);
            if (motorData[3][motor][0].size() > samples_per_plot) {
                motorData[3][motor][0].pop_front();
                motorData[3][motor][1].pop_front();
                motorData[3][motor][2].pop_front();
                motorData[3][motor][3].pop_front();
            }
            if (time.size() > samples_per_plot)
                time.pop_front();

            if ((counter++) % 20 == 0) {
                Q_EMIT newData();
            }
            if (counter % 100 == 0) {
                rescale();
            }
        }
    }
    ROS_INFO("stop receiving udp");
}

void VRPuppets::MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving motor status");
    if(msg->id == ui.fpga->value()) {
        ros::Duration delta = (ros::Time::now() - start_time);
        time.push_back(delta.toSec());
        for (uint motor = 0; motor < msg->position.size(); motor++) {
            std::vector<int>::iterator it = std::find(active_motors[msg->id].begin(), active_motors[msg->id].end(),
                                                      motor);
            if (it != active_motors[msg->id].end() && plotMotor[motor]) {
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
            }else{
                motorData[msg->id][motor][0].push_back(std::numeric_limits<double>::quiet_NaN());
                motorData[msg->id][motor][1].push_back(std::numeric_limits<double>::quiet_NaN());
                motorData[msg->id][motor][2].push_back(std::numeric_limits<double>::quiet_NaN());
                motorData[msg->id][motor][3].push_back(std::numeric_limits<double>::quiet_NaN());
                if (motorData[msg->id][motor][0].size() > samples_per_plot) {
                    motorData[msg->id][motor][0].pop_front();
                    motorData[msg->id][motor][1].pop_front();
                    motorData[msg->id][motor][2].pop_front();
                    motorData[msg->id][motor][3].pop_front();
                }
            }
        }
        if (time.size() > samples_per_plot)
            time.pop_front();

        if ((counter++) % 20 == 0) {
            Q_EMIT newData();
        }

        if (counter % 100 == 0) {
            if (msg->power_sense) {
                if(ui.power_sense!=nullptr)
                    ui.power_sense->setStyleSheet("background-color:green;");
            }else {
                if(ui.power_sense!=nullptr)
                    ui.power_sense->setStyleSheet("background-color:red;");
            }
            rescale();
        }
    }
}

void VRPuppets::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.displacement_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
        ui.current_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][3]);
    }

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.displacement_plot->xAxis->rescale();
    ui.current_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.displacement_plot->replot();
    ui.current_plot->replot();
}

void VRPuppets::rescale(){
    double minima[NUMBER_OF_MOTORS_PER_FPGA][4], maxima[NUMBER_OF_MOTORS_PER_FPGA][4];
    uint minimal_motor[4] = {0,0,0,0}, maximal_motor[4] = {0,0,0,0};
    for(uint type=0;type<4;type++) {
        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            minima[motor][type] = 0;
            maxima[motor][type] = 0;
            for (auto val:motorData[ui.fpga->value()][motor][type]) {
                if (val < minima[motor][type])
                    minima[motor][type] = val;
                if (val > maxima[motor][type])
                    maxima[motor][type] = val;
            }
        }

        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
            if (minima[motor][type] <= minima[minimal_motor[type]][type] && plotMotor[motor])
                minimal_motor[type] = motor;
            if (maxima[motor][type] <= maxima[maximal_motor[type]][type] && plotMotor[motor])
                maximal_motor[type] = motor;
        }
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        if (minimal_motor[0] == motor||maximal_motor[0] == motor)
            ui.position_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[1] == motor||maximal_motor[1] == motor)
            ui.velocity_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[2] == motor||maximal_motor[2] == motor)
            ui.displacement_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[3] == motor||maximal_motor[3] == motor)
            ui.current_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        if (minimal_motor[0] != motor||maximal_motor[0] != motor)
            ui.position_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[1] != motor||maximal_motor[1] != motor)
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[2] != motor||maximal_motor[2] != motor)
            ui.displacement_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[3] != motor||maximal_motor[3] != motor)
            ui.current_plot->graph(motor)->rescaleAxes(true);
    }
}

void VRPuppets::plotMotorChanged(){
    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void VRPuppets::toggleAll(){
    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr) {
            plotMotor[i] = !box->isChecked();
            box->setChecked(plotMotor[i]);
        }
    }
}

void VRPuppets::fpgaChanged(int fpga){
    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr) {
            if(find(active_motors[fpga].begin(),active_motors[fpga].end(),i)!=active_motors[fpga].end()) {
                plotMotor[i] = true;
                box->setChecked(true);
            }else{
                plotMotor[i] = false;
                box->setChecked(false);
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(VRPuppets, rqt_gui_cpp::Plugin)
