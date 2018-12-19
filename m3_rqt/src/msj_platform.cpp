#include <m3_rqt/m3_rqt.hpp>
#include <std_msgs/Float32.h>

M3RQT::M3RQT()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("M3RQT");
}

void M3RQT::initPlugin(qt_gui_cpp::PluginContext &context) {
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
        ui.force_plot->addGraph();
        ui.force_plot->graph(motor)->setPen(QPen(color_pallette[motor]));
        char str[20];
        sprintf(str, "motor_%d", motor);
        QCheckBox *box = widget_->findChild<QCheckBox *>(str);
        QObject::connect(box, SIGNAL(stateChanged(int)), this, SLOT(plotMotorChanged()));
        plotMotor[motor] = true;
    }

    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setLabel("pos[m]");
    ui.position_plot->replot();

    ui.velocity_plot->xAxis->setLabel("time[s]");
    ui.velocity_plot->yAxis->setLabel("vel[m/s]");
    ui.velocity_plot->replot();

    ui.force_plot->xAxis->setLabel("time[s]");
    ui.force_plot->yAxis->setLabel("ticks");
    ui.force_plot->replot();

    for (int axis = 0; axis < 3; axis++) {
        ui.magnetic_plot_0->addGraph();
        ui.magnetic_plot_0->graph(axis)->setPen(QPen(color_pallette[axis]));
        ui.magnetic_plot_1->addGraph();
        ui.magnetic_plot_1->graph(axis)->setPen(QPen(color_pallette[axis]));
        ui.magnetic_plot_2->addGraph();
        ui.magnetic_plot_2->graph(axis)->setPen(QPen(color_pallette[axis]));
    }
    ui.magnetic_plot_0->xAxis->setLabel("time[s]");
    ui.magnetic_plot_0->yAxis->setLabel("sensor 0");
    ui.magnetic_plot_1->xAxis->setLabel("time[s]");
    ui.magnetic_plot_1->yAxis->setLabel("sensor 1");
    ui.magnetic_plot_2->xAxis->setLabel("time[s]");
    ui.magnetic_plot_2->yAxis->setLabel("sensor 2");

    ui.joint_space->addGraph();
    ui.joint_space->graph(0)->setPen(QPen(Qt::red));
    ui.joint_space->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui.joint_space->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    ui.joint_space->addGraph();
    ui.joint_space->graph(1)->setPen(QPen(Qt::blue));
    ui.joint_space->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui.joint_space->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 1));
    ui.joint_space->xAxis->setLabel("sphere_axis0");
    ui.joint_space->yAxis->setLabel("sphere_axis1");

    string path = ros::package::getPath("robots");
    path+="/msj_platform/joint_limits.txt";
    FILE*       file = fopen(path.c_str(),"r");
    if (NULL != file) {
        fscanf(file, "%*[^\n]\n", NULL);
        float qx,qy;
        int i =0;
        while(fscanf(file,"%f %f\n",&qx,&qy)==2){
            limits[0].push_back(qx);
            limits[1].push_back(qy);
            cout << qx << "\t" << qy << endl;
            i++;
        }
        printf("read %d joint limit values\n", i);
        ui.joint_space->graph(0)->setData(limits[0],limits[1]);
        ui.joint_space->xAxis->rescale();
        ui.joint_space->yAxis->rescale();
        ui.joint_space->replot();
    }else{
        cout << "could not open " << path << endl;
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "m3_rqt_plugin");
    }

    joint_state = nh->subscribe("/joint_state", 1, &M3RQT::JointState, this);
    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &M3RQT::MotorStatus, this);
    magneticSensor = nh->subscribe("/roboy/middleware/MagneticSensor", 1, &M3RQT::MagneticSensor, this);
    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    sphere_axis0 = nh->advertise<std_msgs::Float32>("/sphere_axis0/sphere_axis0/target", 1);
    sphere_axis1 = nh->advertise<std_msgs::Float32>("/sphere_axis1/sphere_axis1/target", 1);
    sphere_axis2 = nh->advertise<std_msgs::Float32>("/sphere_axis2/sphere_axis2/target", 1);
    emergencyStop = nh->serviceClient<std_srvs::SetBool>("/msj_platform/emergency_stop");
    zero = nh->serviceClient<std_srvs::Empty>("/msj_platform/zero");
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(this, SIGNAL(newJointState()), this, SLOT(plotJointState()));
    QObject::connect(ui.toggle_all, SIGNAL(clicked()), this, SLOT(toggleAll()));
    QObject::connect(ui.fpga, SIGNAL(valueChanged(int)), this, SLOT(fpgaChanged(int)));
    QObject::connect(ui.motor_pos_0, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_1, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_2, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_3, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_4, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_5, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_6, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));
    QObject::connect(ui.motor_pos_7, SIGNAL(valueChanged(int)), this, SLOT(motorPosChanged(int)));

    ui.stop_button->setStyleSheet("background-color: green");
    QObject::connect(ui.stop_button, SIGNAL(clicked()), this, SLOT(stopButtonClicked()));
    QObject::connect(ui.zero, SIGNAL(clicked()), this, SLOT(zeroClicked()));
    QObject::connect(ui.show_magnetic_field, SIGNAL(clicked()), this, SLOT(showMagneticField()));
    QObject::connect(ui.clear_magnetic_field, SIGNAL(clicked()), this, SLOT(clearMagneticField()));
    QObject::connect(ui.system, SIGNAL(clicked()), this, SLOT(calibrateSystem()));

    grid_thread.reset(new std::thread(&M3RQT::gridMap, this));
    grid_thread->detach();

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    start_time = ros::Time::now();

    zero_rot.setIdentity();

    udp_command.reset(new UDPSocket("192.168.0.134",8000,8000));
    udp.reset(new UDPSocket(8001));
    udp_thread.reset(new std::thread(&M3RQT::receiveMotorData, this));
    udp_thread->detach();

}

void M3RQT::shutdownPlugin() {
    motorStatus.shutdown();
}

void M3RQT::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                  qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void M3RQT::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                     const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void M3RQT::MotorStatus(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving motor status");
    if (msg->id == ui.fpga->value()) {
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
            } else {
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
            rescale();
            rescaleMagneticSensors();
        }
    }
}

void M3RQT::MagneticSensor(const roboy_middleware_msgs::MagneticSensor::ConstPtr &msg) {
    int j = 0;
    ros::Duration delta = (ros::Time::now() - start_time);
    time_sensor.push_back(delta.toSec());
    Matrix4d pose;
    if(!getTransform("world","top_estimate",pose))
        ROS_WARN_THROTTLE(10,"is the IMU on?!");
    poseData.push_back(pose);
    for (int i = 0; i < msg->sensor_id.size(); i++) {
        sensorData[msg->sensor_id[i]][0].push_back(msg->x[j]);
        sensorData[msg->sensor_id[i]][1].push_back(msg->y[j]);
        sensorData[msg->sensor_id[i]][2].push_back(msg->z[j]);
        if(show_magnetic_field){
            static int counter = 9999;
            Vector3d mag(msg->x[j],msg->y[j],msg->z[j]);
            mag = pose.topLeftCorner(3,3)*mag*0.01;
            Vector3d origin(0,0,0);
            if(j == 0)
                publishSphere(mag,"world","magnetic_field_sensor_0",counter++,COLOR(1,0,0,1),0.01,0);
            else if(j == 1)
                publishSphere(mag,"world","magnetic_field_sensor_1",counter++,COLOR(0,1,0,1),0.01,0);
            else if(j == 2)
                publishSphere(mag,"world","magnetic_field_sensor_2",counter++,COLOR(0,0,1,1),0.01,0);
        }
        j++;
    }
    if (time_sensor.size() > samples_per_plot) {
        time_sensor.pop_front();
        poseData.pop_front();
        for (int i = 0; i < msg->sensor_id.size(); i++) {
            sensorData[msg->sensor_id[i]][0].pop_front();
            sensorData[msg->sensor_id[i]][1].pop_front();
            sensorData[msg->sensor_id[i]][2].pop_front();
        }
    }
}

void M3RQT::JointState(const roboy_simulation_msgs::JointState::ConstPtr &msg){
    static int counter = 0;
    counter++;
    if(counter%20==0) {
        lock_guard<mutex> lock(mux);
        for (int i = 3; i < msg->q.size(); i++) {
            if (i > 6)
                return;
            q[i-3].push_back(msg->q[i]);
        }
        Q_EMIT newJointState();
    }
}

long M3RQT::closest(QVector<double> const& vec, double value){
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    return distance(vec.begin(),it);
}

int M3RQT::pnpoly(QVector<double> limits_x, QVector<double> limits_y, double testx, double testy){
    int i, j, c = 0;
    for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
        if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
             (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
            c = !c;
    }
    return c;
}

void M3RQT::gridMap(){
    ros::Rate rate(50);
    double min[3] = {0,0,-0.3}, max[3] = {0,0,0.3};
    for(int i=0;i<limits[0].size();i++){
        if(limits[0][i]<min[0])
            min[0] = limits[0][i];
        if(limits[1][i]<min[1])
            min[1] = limits[1][i];
        if(limits[0][i]>max[0])
            max[0] = limits[0][i];
        if(limits[1][i]>max[1])
            max[1] = limits[1][i];
    }
    double target[3] = {min[0],min[1],0};
    bool dir[3] = {false,false,false};
    bool x_first = true;
    while(ros::ok()){
        if(ui.run_grid->isChecked()){
            if(x_first) {
                target[0] += (dir[0] ? -1.0 : 1.0) * 0.001;
                if (target[0] > max[0] || target[0] < min[0]) {
                    dir[0] = !dir[0];
                    target[1] += (dir[1] ? -1.0 : 1.0) * 0.01;
                    if (target[1] > max[1] || target[1] < min[1]) {
                        dir[1] = false;
                        target[1] = min[1];
                        x_first = false;
                    }
                }
            }else{
                target[1] += (dir[1] ? -1.0 : 1.0) * 0.001;
                if (target[1] > max[1] || target[1] < min[1]) {
                    dir[1] = !dir[1];
                    target[0] += (dir[0] ? -1.0 : 1.0) * 0.01;
                    if (target[0] > max[0] || target[0] < min[0]) {
                        dir[0] = false;
                        target[0] = min[0];
                        x_first = true;
                    }
                }
            }
            if(pnpoly(limits[0],limits[1],target[0],target[1])==0){
//                ROS_INFO_THROTTLE(1,"not inside");
            }else{
//                ROS_INFO_THROTTLE(1,"inside");
                std_msgs::Float32 msg;
                msg.data = target[0];
                sphere_axis0.publish(msg);
                msg.data = target[1];
                sphere_axis1.publish(msg);
                if(ui.heading->isChecked()) {
                    if (target[2] > max[2] || target[2] < min[2])
                        dir[2] = !dir[2];
                    target[2] += (dir[2] ? -1.0 : 1.0) * 0.001;
                    msg.data = target[2];
                    sphere_axis2.publish(msg);
                }
                rate.sleep();
            }
        }
    }
}

void M3RQT::receiveMotorData(){
    ROS_INFO("start receiving udp");
    while (ros::ok()) {
//        int bytes_received = ;
        if (udp->receiveUDP()!=0) {//bytes_received == 12
            ros::Duration delta = (ros::Time::now() - start_time);
            time.push_back(delta.toSec());
            for(int i=0;i<3;i++){
                uint32_t data = (uint32_t) ((uint8_t) udp->buf[3+i*4] << 24 | (uint8_t) udp->buf[2+i*4] << 16 |
                                   (uint8_t) udp->buf[1+i*4] << 8 | (uint8_t) udp->buf[0+i*4]);
                float val = unpack754_32(data);
                motorData[ui.fpga->value()][0][i].push_back(val);
            }
            if (motorData[ui.fpga->value()][0][0].size() > samples_per_plot) {
                motorData[ui.fpga->value()][0][0].pop_front();
                motorData[ui.fpga->value()][0][1].pop_front();
                motorData[ui.fpga->value()][0][2].pop_front();
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

void M3RQT::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.force_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
    }

    for (uint axis = 0; axis < 3; axis++) {
        ui.magnetic_plot_0->graph(axis)->setData(time, sensorData[0][axis]);
        ui.magnetic_plot_1->graph(axis)->setData(time, sensorData[1][axis]);
        ui.magnetic_plot_2->graph(axis)->setData(time, sensorData[2][axis]);
    }

    ui.magnetic_plot_0->xAxis->rescale();
    ui.magnetic_plot_1->xAxis->rescale();
    ui.magnetic_plot_2->xAxis->rescale();

    ui.magnetic_plot_0->replot();
    ui.magnetic_plot_1->replot();
    ui.magnetic_plot_2->replot();

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.force_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.force_plot->replot();

    switch(imu_state[0]){
        case 0:
            ui.system->setStyleSheet("background-color: red");
            break;
        case 1:
            ui.system->setStyleSheet("background-color: orange");
            break;
        case 2:
            ui.system->setStyleSheet("background-color: yellow");
            break;
        case 3:
            ui.system->setStyleSheet("background-color: green");
            break;
    }
    switch(imu_state[1]){
        case 0:
            ui.gyro->setStyleSheet("background-color: red");
            break;
        case 1:
            ui.gyro->setStyleSheet("background-color: orange");
            break;
        case 2:
            ui.gyro->setStyleSheet("background-color: yellow");
            break;
        case 3:
            ui.gyro->setStyleSheet("background-color: green");
            break;
    }
    switch(imu_state[2]){
        case 0:
            ui.acc->setStyleSheet("background-color: red");
            break;
        case 1:
            ui.acc->setStyleSheet("background-color: orange");
            break;
        case 2:
            ui.acc->setStyleSheet("background-color: yellow");
            break;
        case 3:
            ui.acc->setStyleSheet("background-color: green");
            break;
    }
    switch(imu_state[3]){
        case 0:
            ui.mag->setStyleSheet("background-color: red");
            break;
        case 1:
            ui.mag->setStyleSheet("background-color: orange");
            break;
        case 2:
            ui.mag->setStyleSheet("background-color: yellow");
            break;
        case 3:
            ui.mag->setStyleSheet("background-color: green");
            break;
    }
}

void M3RQT::plotJointState(){
    ui.joint_space->graph(1)->setData(q[0],q[1]);
    ui.joint_space->xAxis->rescale();
    ui.joint_space->yAxis->rescale();
    ui.joint_space->replot();
}

void M3RQT::rescale() {
    double minima[NUMBER_OF_MOTORS][3], maxima[NUMBER_OF_MOTORS][3];
    uint minimal_motor[3] = {0, 0, 0}, maximal_motor[3] = {0, 0, 0};
    for (uint type = 0; type < 3; type++) {
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
        if (minimal_motor[0] == motor || maximal_motor[0] == motor)
            ui.position_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[1] == motor || maximal_motor[1] == motor)
            ui.velocity_plot->graph(motor)->rescaleAxes();
        if (minimal_motor[2] == motor || maximal_motor[2] == motor)
            ui.force_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] != motor || maximal_motor[0] != motor)
            ui.position_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[1] != motor || maximal_motor[1] != motor)
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[2] != motor || maximal_motor[2] != motor)
            ui.force_plot->graph(motor)->rescaleAxes(true);
    }
}

void M3RQT::rescaleMagneticSensors() {
    double minima[3][3], maxima[3][3];
    uint minimal_axis[3] = {0, 0, 0}, maximal_axis[3] = {0, 0, 0};
    for (uint sensor = 0; sensor < 3; sensor++) {
        for (uint axis = 0; axis < 3; axis++) {
            minima[axis][sensor] = 0;
            maxima[axis][sensor] = 0;
            for (auto val:motorData[ui.fpga->value()][axis][sensor]) {
                if (val < minima[axis][sensor])
                    minima[axis][sensor] = val;
                if (val > maxima[axis][sensor])
                    maxima[axis][sensor] = val;
            }
        }

        for (uint axis = 0; axis < 3; axis++) {
            if (minima[axis][sensor] <= minima[minimal_axis[sensor]][sensor])
                minimal_axis[sensor] = axis;
            if (maxima[axis][sensor] <= maxima[maximal_axis[sensor]][sensor])
                maximal_axis[sensor] = axis;
        }
    }

    for (uint axis = 0; axis < 3; axis++) {
        if (minimal_axis[0] == axis || maximal_axis[0] == axis)
            ui.magnetic_plot_0->graph(axis)->rescaleAxes();
        if (minimal_axis[1] == axis || maximal_axis[1] == axis)
            ui.magnetic_plot_1->graph(axis)->rescaleAxes();
        if (minimal_axis[2] == axis || maximal_axis[2] == axis)
            ui.magnetic_plot_2->graph(axis)->rescaleAxes();
    }

    for (uint axis = 0; axis < 3; axis++) {
        if (minimal_axis[0] != axis || maximal_axis[0] != axis)
            ui.magnetic_plot_0->graph(axis)->rescaleAxes(true);
        if (minimal_axis[1] != axis || maximal_axis[1] != axis)
            ui.magnetic_plot_1->graph(axis)->rescaleAxes(true);
        if (minimal_axis[2] != axis || maximal_axis[2] != axis)
            ui.magnetic_plot_2->graph(axis)->rescaleAxes(true);
    }
}

void M3RQT::plotMotorChanged() {
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_%d", i);
        QCheckBox *box = widget_->findChild<QCheckBox *>(str);
        if (box != nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void M3RQT::toggleAll() {
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_%d", i);
        QCheckBox *box = widget_->findChild<QCheckBox *>(str);
        if (box != nullptr) {
            plotMotor[i] = !box->isChecked();
            box->setChecked(plotMotor[i]);
        }
    }
}

void M3RQT::fpgaChanged(int fpga) {

}

void M3RQT::motorPosChanged(int pos) {
    roboy_middleware_msgs::MotorCommand msg;
    msg.id = ui.fpga->value();
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_pos_%d", i);
        QSlider *slider = widget_->findChild<QSlider *>(str);
        msg.motors.push_back(i);
        msg.set_points.push_back(slider->value());
        float setpoint = slider->value();
        uint32_t data = pack754_32(setpoint);
        memcpy(udp_command->buf,&data,4);
    }
    motorCommand.publish(msg);

    udp_command->numbytes = 4;
    udp_command->sendUDPToClient();
}

void M3RQT::stopButtonClicked() {
    std_srvs::SetBool msg;
    if (ui.stop_button->isChecked()) {
        ui.stop_button->setStyleSheet("background-color: red");
        msg.request.data = 1;
        emergencyStop.call(msg);
    } else {
        ui.stop_button->setStyleSheet("background-color: green");
        msg.request.data = 0;
        emergencyStop.call(msg);
    }
}

void M3RQT::zeroClicked() {
    std_srvs::Empty msg;
    zero.call(msg);
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_pos_%d", i);
        QSlider *slider = widget_->findChild<QSlider *>(str);
        slider->setValue(0);
    }
}

void M3RQT::showMagneticField(){
    show_magnetic_field = ui.show_magnetic_field->isChecked();
}

void M3RQT::clearMagneticField(){
    clearAll();
}

void M3RQT::calibrateSystem(){
    ros::Time start_time = ros::Time::now();
    double min[3] = {0,0,-1}, max[3] = {0,0,1};
    for(int i=0;i<limits[0].size();i++){
        if(limits[0][i]<min[0])
            min[0] = limits[0][i];
        if(limits[1][i]<min[1])
            min[1] = limits[1][i];
        if(limits[0][i]>max[0])
            max[0] = limits[0][i];
        if(limits[1][i]>max[1])
            max[1] = limits[1][i];
    }
    ros::Rate rate(1/3.0);
    while(imu_state[0]!=3 && (ros::Time::now()-start_time).toSec()<60){
        double q0 = rand()/(double)RAND_MAX*(max[0]-min[0])+min[0];
        double q1 = rand()/(double)RAND_MAX*(max[1]-min[1])+min[1];
        double q2 = rand()/(double)RAND_MAX*(max[2]-min[2])+min[2];
        if(pnpoly(limits[0],limits[1],q0,q1)){
            std_msgs::Float32 msg;
            msg.data = q0;
            sphere_axis0.publish(msg);
            msg.data = q1;
            sphere_axis1.publish(msg);
            msg.data = q2;
            sphere_axis2.publish(msg);
            rate.sleep();
        }
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_motor_status, M3RQT, M3RQT, rqt_gui_cpp::Plugin)
