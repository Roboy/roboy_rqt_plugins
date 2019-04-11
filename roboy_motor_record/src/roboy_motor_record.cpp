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

    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyMotorRecord::MotorStatus, this);
    testRigPosition = nh->subscribe("/roboy/middleware/TestRigRelativePosition", 1, &RoboyMotorRecord::TestRigRelativePosition, this);
    testRigWeight = nh->subscribe("/roboy/middleware/LoadCells", 1, &RoboyMotorRecord::TestRigWeight, this);
    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(control_mode()));
    QObject::connect(ui.record, SIGNAL(toggled(bool)), this, SLOT(record(bool)));
    QObject::connect(ui.motorIDs, SIGNAL(editingFinished()), this, SLOT(motors()));
    QObject::connect(ui.time_span, SIGNAL(editingFinished()), this, SLOT(time_span()));
    control_mode();
    motors();
    ts = ui.time_span->text().toInt();

}

void RoboyMotorRecord::shutdownPlugin() {
}

void RoboyMotorRecord::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyMotorRecord::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyMotorRecord::record(bool checked){
    if(checked) {
        ui.record->setStyleSheet("background-color: red");
        string filename = QFileDialog::getSaveFileName(widget_,
                                                       tr("Choose File Name"), "",
                                                       tr("All Files (*)")).toStdString();
        file.open(filename);
        ROS_INFO("opening %s for logging", filename.c_str());
        if (file.is_open()) {
            ROS_INFO("start recording");
            lock_guard<mutex> lock(mux);
            file << "control mode (0:Position, 1:Velocity, 2:Displacement): " << cm << "\t" << "motors: " << ui.motorIDs->text().toStdString() << endl;
            file << "timestamp[ms]" << "\t";
            if(ui.pos_record->isChecked())
                file << "position[ticks]" << "\t";
            if(ui.vel_record->isChecked())
                file << "velocity[ticks/s]" << "\t";
            if(ui.dis_record->isChecked())
                file << "displacement[ticks]" << "\t";
            if(ui.cur_record->isChecked())
                file << "current[mA]" << "\t";
            if(ui.pwm_record->isChecked())
                file << "pwm[1]" << "\t";
            if(ui.test_rig_pos_record->isChecked())
                file << "test_rig_position[m]" << "\t";
            if(ui.test_rig_weight_record->isChecked())
                file << "test_rig_weight[N]" << "\t";
            file << endl;
            sample = 0;
            roboy_middleware_msgs::MotorCommand msg;
            msg.id = ui.fpga->value();
            for(int fpga=0;fpga<NUMBER_OF_FPGAS;fpga++){
                for(int motor=0;motor<NUMBER_OF_MOTORS_PER_FPGA;motor++){
                    motorData[fpga][motor][POSITION].clear();
                    motorData[fpga][motor][VELOCITY].clear();
                    motorData[fpga][motor][DISPLACEMENT].clear();
                    motorData[fpga][motor][CURRENT].clear();
                    motorData[fpga][motor][DIRECT_PWM].clear();
                    motorData[fpga][motor][7].clear();
                    motorData[fpga][motor][8].clear();
                    msg.motors.push_back(motor);
                    msg.set_points.push_back(ui.setPoint->text().toInt());
                }
            }
            recording = true;
            motorCommand.publish(msg);
            start_time = high_resolution_clock::now();
        } else {
            ROS_ERROR("could not open file");
            ui.record->setStyleSheet("background-color: white");
            recording = false;
        }
    }else {
        ui.record->setStyleSheet("background-color: white");
        ROS_INFO("saving to file recording");
        recording = false;
        file.close();
        roboy_middleware_msgs::MotorCommand msg;
        msg.id = ui.fpga->value();
        int setpoint = ui.setPoint->text().toInt();
        ros::Rate rate(20);
        for(int i=0;i<100;i++) {
            setpoint*=0.95;
            for (int motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
                msg.motors.push_back(motor);
                msg.set_points.push_back(setpoint);
            }
            motorCommand.publish(msg);
            rate.sleep();
        }
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
}

void RoboyMotorRecord::MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg) {
    if(recording && msg->id==ui.fpga->value()) {
        ROS_DEBUG_THROTTLE(5, "receiving motor status");
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        milliseconds time_span = duration_cast<milliseconds>(t1-start_time);
        if(time_span.count()<ts) {
            time.push_back(time_span.count());
            for (uint motor = 0; motor < msg->position.size(); motor++) {
                motorData[msg->id][motor][POSITION].push_back(msg->position[motor]);
                motorData[msg->id][motor][VELOCITY].push_back(msg->velocity[motor]);
                motorData[msg->id][motor][DISPLACEMENT].push_back(msg->displacement[motor]);
                motorData[msg->id][motor][CURRENT].push_back(msg->current[motor]);
                motorData[msg->id][motor][DIRECT_PWM].push_back(msg->pwm_ref[motor]);
                motorData[msg->id][motor][7].push_back(test_rig_pos);
                motorData[msg->id][motor][8].push_back(test_rig_weight);
            }
            lock_guard<mutex> lock(mux);
            file << time_span.count() << "\t";
            for (int i = 0; i < motorIDs.size(); i++) {
                if(ui.pos_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][POSITION].back() << "\t";
                if(ui.vel_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][VELOCITY].back() << "\t";
                if(ui.dis_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][DISPLACEMENT].back() << "\t";
                if(ui.cur_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][CURRENT].back() << "\t";
                if(ui.pwm_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][DIRECT_PWM].back() << "\t";
                if(ui.test_rig_pos_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][7].back() << "\t";
                if(ui.test_rig_weight_record->isChecked())
                    file << motorData[msg->id][motorIDs[i]][8].back() << "\t";
            }
            file << endl;
            Q_EMIT newData();
        }else{
            ui.record->setChecked(false);
        }
    }
}

void RoboyMotorRecord::TestRigRelativePosition(const std_msgs::Float32::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    test_rig_pos = msg->data;
}

void RoboyMotorRecord::TestRigWeight(const roboy_middleware_msgs::ADCvalue::ConstPtr &msg) {
    lock_guard<mutex> lock(mux);
    test_rig_weight = msg->load[0];
}

void RoboyMotorRecord::plotData() {
    vector<double> minimum(motorIDs.size(),0), maximum(motorIDs.size(),0);
    for (uint i = 0; i < motorIDs.size(); i++) {
        ui.plot->graph(motorIDs[i])->setData(time, motorData[ui.fpga->value()][motorIDs[i]][cm]);
//        if(motorData[ui.fpga->value()][motorIDs[i]][cm].back()<minimum[i])
//            minimum[i] = motorData[ui.fpga->value()][motorIDs[i]][cm].back();
//        if(motorData[ui.fpga->value()][motorIDs[i]][cm].back()>maximum[i])
//            maximum[i] = motorData[ui.fpga->value()][motorIDs[i]][cm].back();
        ui.plot->graph(motorIDs[i])->rescaleAxes();
    }
    ui.plot->replot();
}

PLUGINLIB_EXPORT_CLASS(RoboyMotorRecord, rqt_gui_cpp::Plugin)
