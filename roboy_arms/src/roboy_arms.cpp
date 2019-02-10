#include <roboy_arms/roboy_arms.hpp>
#include <QtWidgets/QCheckBox>

RoboyArms::RoboyArms()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyArms");
}

void RoboyArms::initPlugin(qt_gui_cpp::PluginContext &context) {
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
        ros::init(argc, argv, "arms_rqt_plugin");
    }

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.toggle_all, SIGNAL(clicked()), this, SLOT(toggleAll()));
    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(fpgaChanged(int)));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();

    /* create a UDP socket for the left arm*/
    if ((fd[LEFT] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_ERROR("cannot create socket");
    }
//    /* create a UDP socket for the left arm*/
//    if ((fd[RIGHT] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
//        ROS_ERROR("cannot create socket");
//    }

    /* bind the socket to any valid IP address and a specific port */
    memset((char *) &myaddr[LEFT], 0, sizeof(myaddr[LEFT]));
    myaddr[LEFT].sin_family = AF_INET;
    myaddr[LEFT].sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr[LEFT].sin_port = htons(LEFT_PORT);

    /* bind the socket to any valid IP address and a specific port */
    memset((char *) &myaddr[RIGHT], 0, sizeof(myaddr[RIGHT]));
    myaddr[RIGHT].sin_family = AF_INET;
    myaddr[RIGHT].sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr[RIGHT].sin_port = htons(RIGHT_PORT);

    if (bind(fd[LEFT], (struct sockaddr *) &myaddr[LEFT], sizeof(myaddr[LEFT])) < 0) {
        ROS_ERROR("bind for left arm socket failed");
        arm_available[LEFT] = false;
    }else{
        arm_available[LEFT] = true;
    }
    if (bind(fd[RIGHT], (struct sockaddr *) &myaddr[RIGHT], sizeof(myaddr[RIGHT])) < 0) {
        ROS_ERROR("bind for right arm socket failed");
        arm_available[RIGHT] = false;
    }else{
        arm_available[RIGHT] = true;
    }

    arm_control_thread.reset(new std::thread(&RoboyArms::MotorStatus,this));
    arm_control_thread->detach();

}

RoboyArms::~RoboyArms(){
    if(arm_available[LEFT])
        close(fd[LEFT]);
    if(arm_available[RIGHT])
        close(fd[RIGHT]);
}

void RoboyArms::shutdownPlugin() {
    motorStatus.shutdown();
}

void RoboyArms::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyArms::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyArms::MotorStatus() {
    while(ros::ok()) {
        if(arm_available[LEFT]) {
            recvlen[LEFT] = recvfrom(fd[LEFT], buf[LEFT], BUFSIZE, 0, (struct sockaddr *) &remaddr[LEFT], &addrlen);
            if(recvlen[LEFT]==48) {
                ros::Duration delta = (ros::Time::now() - start_time);
                time.push_back(delta.toSec());
                for (uint motor = 0; motor < 4; motor++) {
                    if (plotMotor[motor]) {
                        int32_t pos = (int32_t)(buf[LEFT][motor*4+3]<<24|buf[LEFT][motor*4+2]<<16|buf[LEFT][motor*4+1]<<8|buf[LEFT][motor*4+0]);
                        int16_t vel = (int16_t)(buf[LEFT][16+motor*2+1]<<8|buf[LEFT][16+motor*2+0]);
                        int16_t dis = (int16_t)(buf[LEFT][24+motor*2+1]<<8|buf[LEFT][24+motor*2+0]);
                        int16_t cur = (int16_t)(buf[LEFT][32+motor*2+1]<<8|buf[LEFT][32+motor*2+0]);

                        motorData[LEFT][motor][0].push_back(pos);
                        motorData[LEFT][motor][1].push_back(vel);
                        motorData[LEFT][motor][2].push_back(dis);
                        motorData[LEFT][motor][3].push_back(cur);
                        if (motorData[LEFT][motor][0].size() > samples_per_plot) {
                            motorData[LEFT][motor][0].pop_front();
                            motorData[LEFT][motor][1].pop_front();
                            motorData[LEFT][motor][2].pop_front();
                            motorData[LEFT][motor][3].pop_front();
                        }
                    }else{
                        motorData[LEFT][motor][0].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[LEFT][motor][1].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[LEFT][motor][2].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[LEFT][motor][3].push_back(std::numeric_limits<double>::quiet_NaN());
                        if (motorData[LEFT][motor][0].size() > samples_per_plot) {
                            motorData[LEFT][motor][0].pop_front();
                            motorData[LEFT][motor][1].pop_front();
                            motorData[LEFT][motor][2].pop_front();
                            motorData[LEFT][motor][3].pop_front();
                        }
                    }
                }
                if (time.size() > samples_per_plot)
                    time.pop_front();

                if ((counter++) % 1 == 0) {
                    Q_EMIT newData();
                }

                if (counter % 100 == 0) {
//                if (msg->power_sense) {
//                    if(ui.power_sense!=nullptr)
//                        ui.power_sense->setStyleSheet("background-color:green;");
//                }else {
//                    if(ui.power_sense!=nullptr)
//                        ui.power_sense->setStyleSheet("background-color:red;");
//                }
                    rescale();
                }
                sendto(fd[LEFT], buf[LEFT], 86, 0, (struct sockaddr *) &remaddr[LEFT], addrlen);
            }
        }
        if(arm_available[RIGHT]) {
            recvlen[RIGHT] = recvfrom(fd[RIGHT], buf[RIGHT], BUFSIZE, 0, (struct sockaddr *) &remaddr[RIGHT], &addrlen);
            if(recvlen[RIGHT]==48) {
                ros::Duration delta = (ros::Time::now() - start_time);
                time.push_back(delta.toSec());
                for (uint motor = 0; motor < 4; motor++) {
                    if (plotMotor[motor]) {
                        int32_t pos = (int32_t)(buf[RIGHT][motor*4+3]<<24|buf[RIGHT][motor*4+2]<<16|buf[RIGHT][motor*4+1]<<8|buf[RIGHT][motor*4+0]);
                        int16_t vel = (int16_t)(buf[RIGHT][16+motor*2+1]<<8|buf[RIGHT][16+motor*2+0]);
                        int16_t dis = (int16_t)(buf[RIGHT][24+motor*2+1]<<8|buf[RIGHT][24+motor*2+0]);
                        int16_t cur = (int16_t)(buf[RIGHT][32+motor*2+1]<<8|buf[RIGHT][32+motor*2+0]);

                        motorData[RIGHT][motor][0].push_back(pos);
                        motorData[RIGHT][motor][1].push_back(vel);
                        motorData[RIGHT][motor][2].push_back(dis);
                        motorData[RIGHT][motor][3].push_back(cur);
                        if (motorData[RIGHT][motor][0].size() > samples_per_plot) {
                            motorData[RIGHT][motor][0].pop_front();
                            motorData[RIGHT][motor][1].pop_front();
                            motorData[RIGHT][motor][2].pop_front();
                            motorData[RIGHT][motor][3].pop_front();
                        }
                    }else{
                        motorData[RIGHT][motor][0].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[RIGHT][motor][1].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[RIGHT][motor][2].push_back(std::numeric_limits<double>::quiet_NaN());
                        motorData[RIGHT][motor][3].push_back(std::numeric_limits<double>::quiet_NaN());
                        if (motorData[RIGHT][motor][0].size() > samples_per_plot) {
                            motorData[RIGHT][motor][0].pop_front();
                            motorData[RIGHT][motor][1].pop_front();
                            motorData[RIGHT][motor][2].pop_front();
                            motorData[RIGHT][motor][3].pop_front();
                        }
                    }
                }
                if (time.size() > samples_per_plot)
                    time.pop_front();

                if ((counter++) % 1 == 0) {
                    Q_EMIT newData();
                }

                if (counter % 100 == 0) {
    //                if (msg->power_sense) {
    //                    if(ui.power_sense!=nullptr)
    //                        ui.power_sense->setStyleSheet("background-color:green;");
    //                }else {
    //                    if(ui.power_sense!=nullptr)
    //                        ui.power_sense->setStyleSheet("background-color:red;");
    //                }
                    rescale();
                }
                sendto(fd[RIGHT], buf[RIGHT], 86, 0, (struct sockaddr *) &remaddr[RIGHT], addrlen);
            }
        }
    }
}

void RoboyArms::plotData() {
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

void RoboyArms::rescale(){
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

void RoboyArms::plotMotorChanged(){
    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void RoboyArms::toggleAll(){
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

void RoboyArms::fpgaChanged(int fpga){
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

PLUGINLIB_EXPORT_CLASS(RoboyArms, rqt_gui_cpp::Plugin)
