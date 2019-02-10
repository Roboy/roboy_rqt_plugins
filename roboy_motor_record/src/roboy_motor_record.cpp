#include <roboy_motor_record/roboy_motor_record.hpp>

RoboyMotorRecord::RoboyMotorRecord()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyMotorRecord");
}

void RoboyMotorRecord::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        ui.plot->addGraph();
        ui.plot->graph(motor)->setPen(QPen(color_pallette[motor]));
    }
    ui.plot->xAxis->setLabel("time[ms]");

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_record_rqt_plugin");
    }

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 10, &RoboyMotorRecord::MotorStatus, this);
    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.force, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.record, SIGNAL(clicked()), this, SLOT(record()));
    QObject::connect(ui.motorIDs, SIGNAL(editingFinished()), this, SLOT(motors()));
    QObject::connect(ui.time_span, SIGNAL(editingFinished()), this, SLOT(time_span()));

    ts = ui.time_span->text().toInt();
}

void RoboyMotorRecord::shutdownPlugin() {
    motorStatus.shutdown();
}

void RoboyMotorRecord::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorRecord::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorRecord::record(){
    if(ui.record->isChecked()) {
        ui.record->setStyleSheet("background-color: red");
        char str[100], t[20];
        time_t now = std::time(0);
        strftime(t, 20, "%d%m%Y_%H%M%S", localtime(&now));
        sprintf(str, "record_%s.log", t);
        file.open(str);
        if (file.is_open()) {
            ROS_INFO("start recording");
            lock_guard<mutex> lock(mux);
            file << "timestamp, \t" << ui.motorIDs->text().toStdString() << "control mode (0:Position, 1:Velocity, 2:Displacement): " << cm << endl;
            sample = 0;
            roboy_middleware_msgs::MotorCommand msg;
            msg.id = ui.fpga->value();
            for(int fpga=0;fpga<NUMBER_OF_FPGAS;fpga++){
                for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
                    motorData[fpga][motor][POSITION].clear();
                    motorData[fpga][motor][VELOCITY].clear();
                    motorData[fpga][motor][DISPLACEMENT].clear();
                    motorData[fpga][motor][FORCE].clear();
                    msg.motors.push_back(motor);
                    msg.set_points.push_back(ui.setPoint->text().toInt());
                }
            }
            recording = true;
            motorCommand.publish(msg);
            start_time = high_resolution_clock::now();
        } else {
            ROS_ERROR("could not open file");
        }
    }else {
        ui.record->setStyleSheet("background-color: white");
        ROS_INFO("saving to file recording");
        recording = false;
        file.close();
    }
}

void RoboyMotorRecord::motors(){
    motorIDs.clear();
    stringstream ss(ui.motorIDs->text().toStdString());
    int i;
    while (ss >> i)
    {
        motorIDs.push_back(i);

        if (ss.peek() == ',')
            ss.ignore();
    }

    for (i=0; i< motorIDs.size(); i++)
        cout << motorIDs.at(i)<<endl;
}

void RoboyMotorRecord::time_span(){
    ts = ui.time_span->text().toInt();
}

void RoboyMotorRecord::control_mode(){
    if(ui.pos->isChecked()){
        ui.plot->yAxis->setLabel("ticks");
        ui.plot->replot();
        cm = POSITION;
    }
    if(ui.vel->isChecked()){
        ui.plot->yAxis->setLabel("ticks/s");
        ui.plot->replot();
        cm = VELOCITY;
    }
    if(ui.dis->isChecked()){
        ui.plot->yAxis->setLabel("ticks");
        ui.plot->replot();
        cm = DISPLACEMENT;
    }
    if(ui.force->isChecked()){
        ui.plot->yAxis->setLabel("N");
        ui.plot->replot();
        cm = FORCE;
    }
}

void RoboyMotorRecord::MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg) {
    if(recording && msg->id==ui.fpga->value()) {
        ROS_DEBUG_THROTTLE(5, "receiving motor status");
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        milliseconds time_span = duration_cast<milliseconds>(t1-start_time);
        if(time_span.count()<ts) {
            time.push_back(time_span.count());
            for (uint motor = 0; motor < msg->position.size(); motor++) {
                motorData[msg->id][motor][0].push_back(msg->position[motor]);
                motorData[msg->id][motor][1].push_back(msg->velocity[motor]);
                motorData[msg->id][motor][2].push_back(msg->displacement[motor]);
                motorData[msg->id][motor][3].push_back(msg->current[motor]);
            }
            lock_guard<mutex> lock(mux);
            file << time_span.count() << ",\t";
            for (int i = 0; i < motorIDs.size(); i++) {
                switch (cm) {
                    case POSITION:
                        file << motorData[msg->id][motorIDs[i]][POSITION].back();
                        break;
                    case VELOCITY:
                        file << motorData[msg->id][motorIDs[i]][VELOCITY].back();
                        break;
                    case DISPLACEMENT:
                        file << motorData[msg->id][motorIDs[i]][DISPLACEMENT].back();
                        break;
                    case FORCE:
                        file << motorData[msg->id][motorIDs[i]][FORCE].back();
                        break;
                }
                if (i < motorIDs.size() - 1)
                    file << ",\t";
            }
            file << endl;
            Q_EMIT newData();
        }else{
            ui.record->setChecked(false);
        }
    }
}

void RoboyMotorRecord::plotData() {
    vector<double> minimum(motorIDs.size(),0), maximum(motorIDs.size(),0);
    for (uint i = 0; i < motorIDs.size(); i++) {
        ui.plot->graph(motorIDs[i])->setData(time, motorData[ui.fpga->value()][motorIDs[i]][cm]);
        if(motorData[ui.fpga->value()][motorIDs[i]][cm].back()<minimum[i])
            minimum[i] = motorData[ui.fpga->value()][motorIDs[i]][cm].back();
        if(motorData[ui.fpga->value()][motorIDs[i]][cm].back()>maximum[i])
            maximum[i] = motorData[ui.fpga->value()][motorIDs[i]][cm].back();
        ui.plot->graph(motorIDs[i])->rescaleAxes();
    }
    ui.plot->replot();
}

PLUGINLIB_EXPORT_CLASS(RoboyMotorRecord, rqt_gui_cpp::Plugin)
