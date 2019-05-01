#include <msj_platform_rqt/msj_platform.hpp>
#include <std_msgs/Float32.h>

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

    ui.angle_plot->xAxis->setLabel("time[s]");
    ui.angle_plot->yAxis->setLabel("ticks");
    ui.angle_plot->replot();

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

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "msj_platform_rqt_plugin");
    }

    joint_state = nh->subscribe("/joint_states_training", 1, &MSJPlatformRQT::JointState, this);
    motorStatus = nh->subscribe("/roboy/middleware/MotorStatus", 1, &MSJPlatformRQT::MotorStatus, this);
    magneticSensor = nh->subscribe("/roboy/middleware/MagneticSensor", 1, &MSJPlatformRQT::MagneticSensor, this);
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
    QObject::connect(ui.measure_joint_limits, SIGNAL(clicked()), this, SLOT(measureJointLimits()));
    QObject::connect(ui.show_magnetic_field, SIGNAL(clicked()), this, SLOT(showMagneticField()));
    QObject::connect(ui.clear_magnetic_field, SIGNAL(clicked()), this, SLOT(clearMagneticField()));
    QObject::connect(ui.system, SIGNAL(clicked()), this, SLOT(calibrateSystem()));

    spinner.reset(new ros::AsyncSpinner(0));
    spinner->start();

    start_time = ros::Time::now();

    zero_rot.setIdentity();

    string path = ros::package::getPath("robots");
    string model_name;
    nh->getParam("model_name", model_name);
    path+="/" + model_name + "/joint_limits.yaml";
    readJointLimits(path);

    grid_thread.reset(new std::thread(&MSJPlatformRQT::gridMap, this));
    grid_thread->detach();

//    uint32_t ip;
//    inet_pton(AF_INET, "192.168.255.255", &ip);
//    udp.reset(new UDPSocket(8000));
//    udp_thread.reset(new std::thread(&MSJPlatformRQT::receivePose, this));
//    udp_thread->detach();
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
                motorData[msg->id][motor][0].push_back(0);//std::numeric_limits<double>::quiet_NaN()
                motorData[msg->id][motor][1].push_back(0);//std::numeric_limits<double>::quiet_NaN()
                motorData[msg->id][motor][2].push_back(0);//std::numeric_limits<double>::quiet_NaN()
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
            rescaleMagneticSensors();
            rescale();
        }
    }
}

void MSJPlatformRQT::MagneticSensor(const roboy_middleware_msgs::MagneticSensor::ConstPtr &msg) {
    int j = 0;
    static int counter = 0;
    ros::Duration delta = (ros::Time::now() - start_time);
    time_sensor.push_back(delta.toSec());
    for (int i = 0; i < msg->sensor_id.size(); i++) {
        sensorData[msg->sensor_id[i]][0].push_back(msg->x[j]);
        sensorData[msg->sensor_id[i]][1].push_back(msg->y[j]);
        sensorData[msg->sensor_id[i]][2].push_back(msg->z[j]);
//        if(show_magnetic_field){
//            static int counter = 9999;
//            Vector3d mag(msg->x[j],msg->y[j],msg->z[j]);
//            mag = pose.topLeftCorner(3,3)*mag*0.01;
//            Vector3d origin(0,0,0);
//            if(j == 0)
//                publishSphere(mag,"world","magnetic_field_sensor_0",counter++,COLOR(1,0,0,1),0.01,0);
//            else if(j == 1)
//                publishSphere(mag,"world","magnetic_field_sensor_1",counter++,COLOR(0,1,0,1),0.01,0);
//            else if(j == 2)
//                publishSphere(mag,"world","magnetic_field_sensor_2",counter++,COLOR(0,0,1,1),0.01,0);
//        }
        j++;
    }
    if (time_sensor.size() > samples_per_plot) {
        time_sensor.pop_front();
//        poseData.pop_front();
        for (int i = 0; i < msg->sensor_id.size(); i++) {
            sensorData[msg->sensor_id[i]][0].pop_front();
            sensorData[msg->sensor_id[i]][1].pop_front();
            sensorData[msg->sensor_id[i]][2].pop_front();
        }
    }
}

void MSJPlatformRQT::JointState(const sensor_msgs::JointState::ConstPtr &msg){
    static int counter = 0;
    counter++;
    if(counter%20==0) {
        lock_guard<mutex> lock(mux);
        for (int i = 0; i < msg->name.size(); i++) {
            q[i].push_back(msg->position[i]);
        }
        if(q[0].size()>10000){
            q[0].pop_front();
            q[1].pop_front();
            q[2].pop_front();
        }
        Q_EMIT newJointState();
        if(ui.measure_joint_limits->isChecked()){
            limits[0].push_back(q[0].back());
            limits[1].push_back(q[1].back());
        }
    }
}

long MSJPlatformRQT::closest(QVector<double> const& vec, double value){
    auto const it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end()) { return -1; }
    return distance(vec.begin(),it);
}

int MSJPlatformRQT::pnpoly(QVector<double> limits_x, QVector<double> limits_y, double testx, double testy){
    int i, j, c = 0;
    for (i = 0, j = limits_x.size()-1; i < limits_x.size(); j = i++) {
        if ( ((limits_y[i]>testy) != (limits_y[j]>testy)) &&
             (testx < (limits_x[j]-limits_x[i]) * (testy-limits_y[i]) / (limits_y[j]-limits_y[i]) + limits_x[i]) )
            c = !c;
    }
    return c;
}

void MSJPlatformRQT::gridMap(){
    ros::Rate rate(50);
    double min[3] = {0,0,-0.5}, max[3] = {0,0,0.5};
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
    ROS_INFO("min: %f %f %f", min[0], min[1], min[2]);
    ROS_INFO("max: %f %f %f", max[0], max[1], max[2]);
    double target[3] = {0,0,0};
    bool dir[3] = {false,false,false};
    bool x_first = true;
    while(ros::ok()){
        if(ui.grid_map->isChecked()){
            double speed_axis0 = ui.speed_axis0->value()/5000.0;
            double speed_axis1 = ui.speed_axis1->value()/5000.0;
            double speed_axis2 = ui.speed_axis2->value()/5000.0;
            if(x_first) {
                bool ok;

                target[0] += (dir[0] ? -1.0 : 1.0) * speed_axis0;
                if (target[0] > max[0] || target[0] < min[0]) {
                    dir[0] = !dir[0];
                    target[1] += (dir[1] ? -1.0 : 1.0) * speed_axis1;
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
                ROS_INFO_THROTTLE(5,"%f %f not inside", target[0], target[1]);
            }else{
                ROS_INFO_THROTTLE(5,"%f %f inside", target[0], target[1]);
                std_msgs::Float32 msg;
                msg.data = target[0];
                sphere_axis0.publish(msg);
                msg.data = target[1];
                sphere_axis1.publish(msg);
                if(ui.heading->isChecked()) {
                    if (target[2] > max[2] || target[2] < min[2])
                        dir[2] = !dir[2];
                    target[2] += (dir[2] ? -1.0 : 1.0) * speed_axis2;
                    msg.data = target[2];
                    sphere_axis2.publish(msg);
                }
                rate.sleep();
            }
        }
    }
}

void MSJPlatformRQT::receivePose(){
    ROS_INFO("start receiving udp");
    while (ros::ok()) {
        int bytes_received = udp->receiveUDP();
        if (bytes_received == 16) {
            float q[4];
            for(int i=0;i<4;i++){
                uint32_t data = (uint32_t) ((uint8_t) udp->buf[3+i*4] << 24 | (uint8_t) udp->buf[2+i*4] << 16 |
                                   (uint8_t) udp->buf[1+i*4] << 8 | (uint8_t) udp->buf[0+i*4]);
                q[i] = unpack754_32(data);
            }
            Quaterniond quat(q[3],q[0],q[1],q[2]);
            if(ui.zero_pose->isChecked()){
                zero_rot = quat;
                ui.zero_pose->setChecked(false);
            }
            quat = quat*zero_rot.inverse();
            geometry_msgs::Pose p, p_top;
            p.orientation.x = quat.x();
            p.orientation.y = quat.y();
            p.orientation.z = quat.z();
            p.orientation.w = quat.w();
            getTransform("world","top",p_top);
            p.position = p_top.position;
            publishTransform("world","top_estimate",p);
        }else if(bytes_received == 4){
            imu_state[0] = udp->buf[0];
            imu_state[1] = udp->buf[1];
            imu_state[2] = udp->buf[2];
            imu_state[3] = udp->buf[3];
        }
    }
    ROS_INFO("stop receiving udp");
}

void MSJPlatformRQT::plotData() {
    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        ui.position_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][0]);
        ui.velocity_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][1]);
        ui.angle_plot->graph(motor)->setData(time, motorData[ui.fpga->value()][motor][2]);
    }

    for (uint axis = 0; axis < 3; axis++) {
        ui.magnetic_plot_0->graph(axis)->setData(time_sensor, sensorData[0][axis]);
        ui.magnetic_plot_1->graph(axis)->setData(time_sensor, sensorData[1][axis]);
        ui.magnetic_plot_2->graph(axis)->setData(time_sensor, sensorData[2][axis]);
    }

    ui.magnetic_plot_0->xAxis->rescale();
    ui.magnetic_plot_1->xAxis->rescale();
    ui.magnetic_plot_2->xAxis->rescale();

    ui.magnetic_plot_0->replot();
    ui.magnetic_plot_1->replot();
    ui.magnetic_plot_2->replot();

    ui.position_plot->xAxis->rescale();
    ui.velocity_plot->xAxis->rescale();
    ui.angle_plot->xAxis->rescale();

    ui.position_plot->replot();
    ui.velocity_plot->replot();
    ui.angle_plot->replot();

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

void MSJPlatformRQT::plotJointState(){
    ui.joint_space->graph(1)->setData(q[0],q[1]);
    ui.joint_space->xAxis->rescale();
    ui.joint_space->yAxis->rescale();
    ui.joint_space->replot();
}

void MSJPlatformRQT::rescale() {
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
            ui.angle_plot->graph(motor)->rescaleAxes();
    }

    for (uint motor = 0; motor < NUMBER_OF_MOTORS; motor++) {
        if (minimal_motor[0] != motor || maximal_motor[0] != motor)
            ui.position_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[1] != motor || maximal_motor[1] != motor)
            ui.velocity_plot->graph(motor)->rescaleAxes(true);
        if (minimal_motor[2] != motor || maximal_motor[2] != motor)
            ui.angle_plot->graph(motor)->rescaleAxes(true);
    }
}

void MSJPlatformRQT::rescaleMagneticSensors() {
    double minima[3][3], maxima[3][3];
    uint minimal_axis[3] = {0, 0, 0}, maximal_axis[3] = {0, 0, 0};
    for (uint sensor = 0; sensor < 3; sensor++) {
        for (uint axis = 0; axis < 3; axis++) {
            minima[axis][sensor] = 0;
            maxima[axis][sensor] = 0;
            for (auto val:motorData[ui.fpga->value()][axis][sensor]) {
                if (val < minima[axis][sensor] && val)
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

void MSJPlatformRQT::plotMotorChanged() {
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_%d", i);
        QCheckBox *box = widget_->findChild<QCheckBox *>(str);
        if (box != nullptr)
            plotMotor[i] = box->isChecked();
    }
}

void MSJPlatformRQT::toggleAll() {
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

void MSJPlatformRQT::fpgaChanged(int fpga) {

}

void MSJPlatformRQT::motorPosChanged(int pos) {
    roboy_middleware_msgs::MotorCommand msg;
    msg.id = ui.fpga->value();
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_pos_%d", i);
        QSlider *slider = widget_->findChild<QSlider *>(str);
        msg.motors.push_back(i);
        msg.set_points.push_back(slider->value());
    }
    motorCommand.publish(msg);
}

void MSJPlatformRQT::stopButtonClicked() {
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

void MSJPlatformRQT::zeroClicked() {
    std_srvs::Empty msg;
    zero.call(msg);
    for (int i = 0; i < NUMBER_OF_MOTORS; i++) {
        char str[20];
        sprintf(str, "motor_pos_%d", i);
        QSlider *slider = widget_->findChild<QSlider *>(str);
        slider->setValue(0);
    }
}

void MSJPlatformRQT::measureJointLimits() {
    if(ui.measure_joint_limits->isChecked()) {
        limits[0].clear();
        limits[1].clear();
    }else{
        string path = ros::package::getPath("robots");
        string model_name;
        nh->getParam("model_name", model_name);
        path+="/" + model_name + "/joint_limits.yaml";
        if(limits[0].size()>0) {
            ROS_INFO("recorded %d joint limits", limits[0].size());
            writeJointLimits(path);
        }else{
            ROS_WARN("did not record any joint limits, are the vive trackers on?");
        }
    }
}

void MSJPlatformRQT::showMagneticField(){
    show_magnetic_field = ui.show_magnetic_field->isChecked();
}

void MSJPlatformRQT::clearMagneticField(){
    clearAll();
}

void MSJPlatformRQT::calibrateSystem(){
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

bool MSJPlatformRQT::readJointLimits(const string &filepath){
    if(!fileExists(filepath)) {
        ROS_ERROR_STREAM(filepath << " does not exist, check your path");
        return false;
    }
    YAML::Node config = YAML::LoadFile(filepath);
    vector<double> sphere_axis0_limits = config["sphere_axis0"].as<vector<double>>();
    vector<double> sphere_axis1_limits = config["sphere_axis1"].as<vector<double>>();
    if(sphere_axis0_limits.size()!=sphere_axis1_limits.size()){
        ROS_ERROR("not the same amount of joint limits for sphere_axis0 and sphere_axis1, aborting...");
        return false;
    }
    limits[0].clear();
    limits[1].clear();
    for(auto val:sphere_axis0_limits)
        limits[0].push_back(val);
    for(auto val:sphere_axis1_limits)
        limits[1].push_back(val);
    ROS_INFO("read %ld joint limits", sphere_axis0_limits.size());
    ui.joint_space->graph(0)->setData(limits[0],limits[1]);
    ui.joint_space->xAxis->rescale();
    ui.joint_space->yAxis->rescale();
    ui.joint_space->replot();
    return true;
}

bool MSJPlatformRQT::writeJointLimits(const string &filepath){
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }
    YAML::Node config;
    {
        stringstream str;
        str << "[";
        for (uint i = 0; i < limits[0].size(); i++) {
            if (i < limits[0].size() - 1)
                str << "0,";
            else
                str << "0]";
        }
        YAML::Node node = YAML::Load(str.str());
        int i = 0;
        for (auto val:limits[0]) {
            node[i] = val;
            i++;
        }
        config["sphere_axis0"] = node;
    }
    {
        stringstream str;
        str << "[";
        for (uint i = 0; i < limits[1].size(); i++) {
            if (i < limits[1].size() - 1)
                str << "0,";
            else
                str << "0]";
        }
        YAML::Node node = YAML::Load(str.str());
        int i = 0;
        for (auto val:limits[1]) {
            node[i] = val;
            i++;
        }
        config["sphere_axis1"] = node;
    }

    fout << config;
    return true;
}

bool MSJPlatformRQT::fileExists(const string &filepath){
    struct stat buffer;
    return (stat (filepath.c_str(), &buffer) == 0);
}

PLUGINLIB_EXPORT_CLASS(MSJPlatformRQT, rqt_gui_cpp::Plugin)
