#include <msj_platform_rqt/msj_platform_rqt.hpp>

MSJPlatformRQT::MSJPlatformRQT()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("MSJPlatformRQT");
}

void MSJPlatformRQT::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.position_plot->addGraph();
        ui.position_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.velocity_plot->addGraph();
        ui.velocity_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        ui.angle_plot->addGraph();
        ui.angle_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        char str[20];
        sprintf(str,"motor_%d",motor);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        QObject::connect(box, SIGNAL(stateChanged(int)), this, SLOT(plotMotorChanged()));
        plotMotor[motor] = true;
    }
    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setLabel("pos[m]");
    ui.position_plot->replot();

    ui.velocity_plot->xAxis->setLabel("time[s]");
    ui.velocity_plot->yAxis->setLabel("vel[m/s]");
    ui.velocity_plot->replot();

    ui.angle_plot->xAxis->setLabel("time[s]");
    ui.angle_plot->yAxis->setLabel("ticks");
    ui.angle_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "msj_platform_rqt_plugin");
    }

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &MSJPlatformRQT::MotorStatus, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.toggle_all, SIGNAL(clicked()), this, SLOT(toggleAll()));
    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(fpgaChanged(int)));


    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();
}

void MSJPlatformRQT::shutdownPlugin() {
    motorStatus.shutdown();
}

void MSJPlatformRQT::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void MSJPlatformRQT::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void MSJPlatformRQT::MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving motor status");
    if(msg->id == ui.fpga->value()) {
        ros::Duration delta = (ros::Time::now() - start_time);
        time.push_back(delta.toSec());
        for (uint motor = 0; motor < msg->position.size(); motor++) {
            if (plotMotor[motor]) {
                motorData[msg->id][motor][0].push_back(msjMeterPerEncoderTick(msg->position[motor]));
                motorData[msg->id][motor][1].push_back(msjMeterPerEncoderTick(msg->velocity[motor]));
                motorData[msg->id][motor][2].push_back(msg->angle[motor]);
                if (motorData[msg->id][motor][0].size() > samples_per_plot) {
                    motorData[msg->id][motor][0].pop_front();
                    motorData[msg->id][motor][1].pop_front();
                    motorData[msg->id][motor][2].pop_front();
                }
            }else{
                motorData[msg->id][motor][0].push_back(std::numeric_limits<double>::quiet_NaN());
                motorData[msg->id][motor][1].push_back(std::numeric_limits<double>::quiet_NaN());
                motorData[msg->id][motor][2].push_back(std::numeric_limits<double>::quiet_NaN());
                if (motorData[msg->id][motor][0].size() > samples_per_plot) {
                    motorData[msg->id][motor][0].pop_front();
                    motorData[msg->id][motor][1].pop_front();
                    motorData[msg->id][motor][2].pop_front();
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

void MSJPlatformRQT::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.angle_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
    }

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.angle_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.angle_plot->replot();
}

void MSJPlatformRQT::rescale(){
    double minima[NUMBER_OF_MOTORS][3], maxima[NUMBER_OF_MOTORS][3];
    uint minimal_motor[3] = {0,0,0}, maximal_motor[3] = {0,0,0};
    for(uint type=0;type<3;type++) {
        for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
            minima[motor][type] = 0;
            maxima[motor][type] = 0;
            for (auto val:motorData[ui.fpga->value()][motor][type]) {
                if (val < minima[motor][type])
                    minima[motor][type] = val;
                if (val > maxima[motor][type])
                    maxima[motor][type] = val;
            }
        }

        for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
            if (minima[motor][type] <= minima[minimal_motor[type]][type] && plotMotor[motor])
                minimal_motor[type] = motor;
            if (maxima[motor][type] <= maxima[maximal_motor[type]][type] && plotMotor[motor])
                maximal_motor[type] = motor;
        }
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] == motor||maximal_motor[0] == motor)
            ui.position_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[1] == motor||maximal_motor[1] == motor)
            ui.velocity_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[2] == motor||maximal_motor[2] == motor)
            ui.angle_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] != motor||maximal_motor[0] != motor)
            ui.position_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[1] != motor||maximal_motor[1] != motor)
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[2] != motor||maximal_motor[2] != motor)
            ui.angle_plot->graph(motor)->rescaleAxes(true);
    }
}

void MSJPlatformRQT::plotMotorChanged(){
    for(int i=0;i<NUMBER_OF_MOTORS;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void MSJPlatformRQT::toggleAll(){
    for(int i=0;i<NUMBER_OF_MOTORS;i++) {
        char str[20];
        sprintf(str,"motor_%d",i);
        QCheckBox *box = widget_->findChild<QCheckBox*>(str);
        if(box!=nullptr) {
            plotMotor[i] = !box->isChecked();
            box->setChecked(plotMotor[i]);
        }
    }
}

void MSJPlatformRQT::fpgaChanged(int fpga){
    for(int i=0;i<NUMBER_OF_MOTORS;i++) {
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

PLUGINLIB_DECLARE_CLASS(roboy_motor_status, MSJPlatformRQT, MSJPlatformRQT, rqt_gui_cpp::Plugin)
