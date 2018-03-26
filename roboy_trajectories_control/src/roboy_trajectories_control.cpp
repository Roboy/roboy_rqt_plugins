#include <roboy_trajectories_control/roboy_trajectories_control.hpp>


RoboyTrajectoriesControl::RoboyTrajectoriesControl()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyTrajectoriesControl");
}

void RoboyTrajectoriesControl::initPlugin(qt_gui_cpp::PluginContext &context) {

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    checkMotorStatus();
    pullExistingTrajectories();

    QObject::connect(ui.clearBehavior, SIGNAL(clicked()), this, SLOT(clearAllTrajectoriesButtonClicked()));
    QObject::connect(ui.playBehavior, SIGNAL(clicked()), this, SLOT(playTrajectoriesButtonClicked()));
    QObject::connect(ui.refreshTrajectories, SIGNAL(clicked()), this, SLOT(refreshTrajectoriesButtonClicked()));
    QObject::connect(ui.addPause, SIGNAL(clicked()), this, SLOT(addPauseButtonClicked()));
    QObject::connect(ui.relaxAll, SIGNAL(clicked()), this, SLOT(relaxAllMusclesButtonClicked()));
    QObject::connect(ui.startInit, SIGNAL(clicked()), this, SLOT(startInitializationButtonClicked()));
    QObject::connect(ui.startRecord, SIGNAL(clicked()), this, SLOT(startRecordTrajectoryButtonClicked()));
    QObject::connect(ui.stopRecord, SIGNAL(clicked()), this, SLOT(stopRecordTrajectoryButtonClicked()));
    QObject::connect(ui.pauseDuration, SIGNAL(valueChanged(int)), this, SLOT(setPauseDuration(int)));

    ui.playBehavior->setEnabled(false);
    ui.clearBehavior->setEnabled(false);
    ui.addPause->setEnabled(false);
    ui.stopRecord->setEnabled(false);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "trajectories_control_rqt_plugin");
    }

//    motorControlServiceClient = nh->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/middleware/ControlMode");
    emergencyStopServiceClient = nh->serviceClient<std_srvs::SetBool>("/roboy/middleware/EmergencyStop");
    setDisplacementForAllServiceClient = nh->serviceClient<roboy_communication_middleware::SetInt16>("/roboy/middleware/SetDisplacementForAll");
    startRecordTrajectoryServiceClient = nh->serviceClient<roboy_communication_control::StartRecordTrajectory>("/roboy/middleware/StartRecordTrajectory");
    stopRecordTrajectoryServiceClient = nh->serviceClient<roboy_communication_control::StopRecordTrajectory>("/roboy/middleware/StopRecordTrajectory");
    performMovementServiceClient = nh->serviceClient<roboy_communication_control::PerformMovement>("/roboy/middleware/ReplayTrajectory");
    executeBehaviorServiceClient = nh->serviceClient<roboy_communication_control::PerformBehavior>("/roboy/middleware/ExecuteBehavior");
    listExistingTrajectoriesServiceClient = nh->serviceClient<roboy_communication_control::ListTrajectories>("roboy/middleware/ListExistingTrajectories");

//    motorCommandPublisher = nh->advertise<roboy_communication_middleware::motorCommand>("/roboy/middleware/MotorCommand", 1);
    motorStatusSubscriber = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyTrajectoriesControl::motorStatusCallback, this);


//    scene->addEllipse(0,0,50,50,outlinePen,greenBrush);

//    scrollArea->setBackgroundRole(QPalette::Window);
//    scrollArea->setFrameShadow(QFrame::Plain);
//    scrollArea->setFrameShape(QFrame::NoFrame);
//    scrollArea->setWidgetResizable(true);
//
//    //vertical box that contains all the checkboxes for the filters
//    QWidget* motor_command_scrollarea = new QWidget(widget_);
//    motor_command_scrollarea->setObjectName("motor_command_scrollarea");
//    motor_command_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
//    motor_command_scrollarea->setLayout(new QVBoxLayout(motor_command_scrollarea));
//    scrollArea->setWidget(motor_command_scrollarea);
//
//    nh = ros::NodeHandlePtr(new ros::NodeHandle);
//    if (!ros::isInitialized()) {
//        int argc = 0;
//        char **argv = NULL;
//        ros::init(argc, argv, "motor_command_rqt_plugin");
//    }
//
//    if(nh->hasParam("number_of_fpgas")){
//        nh->getParam("number_of_fpgas",number_of_fpgas);
//        ROS_INFO("found number_of_fpgas %d on parameter server", number_of_fpgas);
//    }
//
//    total_number_of_motors = number_of_fpgas*NUMBER_OF_MOTORS_PER_FPGA;
//
//    for(uint fpga = 0; fpga<number_of_fpgas; fpga++) {
//        for (uint motor = 0; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
//            QWidget *widget = new QWidget(motor_command_scrollarea);
//            char str[100];
//            sprintf(str, "motor%d_%d", fpga, motor);
//            widget->setObjectName(str);
//            widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
//            widget->setLayout(new QHBoxLayout(widget));
//
//            QLabel *label = new QLabel(widget);
//            sprintf(str, "%d/%d", fpga, motor);
//            label->setFixedSize(30,30);
//            label->setText(str);
//            widget->layout()->addWidget(label);
//
//            QRadioButton *p = new QRadioButton(widget);
//            p->setText("pos");
//            p->setFixedSize(60,30);
//            p->setCheckable(true);
//            p->setObjectName("pos");
//            pos.push_back(p);
//            QObject::connect(p, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//            widget->layout()->addWidget(p);
//
//            QRadioButton *v = new QRadioButton(widget);
//            v->setText("vel");
//            v->setFixedSize(60,30);
//            v->setCheckable(true);
//            v->setObjectName("vel");
//            widget->layout()->addWidget(v);
//            vel.push_back(v);
//            QObject::connect(v, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//            QRadioButton *d = new QRadioButton(widget);
//            d->setText("dis");
//            d->setFixedSize(60,30);
//            d->setCheckable(true);
//            d->setObjectName("dis");
//            d->setChecked(true);
//            setpoint.push_back(0);
//            control_mode.push_back(DISPLACEMENT);
//            widget->layout()->addWidget(d);
//            dis.push_back(d);
//            QObject::connect(d, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//            QRadioButton *f = new QRadioButton(widget);
//            f->setText("force");
//            f->setFixedSize(60,30);
//            f->setCheckable(true);
//            f->setObjectName("force");
//            widget->layout()->addWidget(f);
//            force.push_back(f);
//            QObject::connect(f, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//            QLineEdit *line = new QLineEdit(widget);
//            line->setFixedSize(100,30);
//            widget->layout()->addWidget(line);
//            setpoint_widget.push_back(line);
//            QObject::connect(line, SIGNAL(editingFinished()), this, SLOT(setPointChanged()));
//            setpoint_widget.back()->setText(QString::number(setpoint[motor]));
//
//            QSlider *slider = new QSlider(Qt::Orientation::Horizontal,widget);
//            slider->setFixedSize(100,30);
//            slider->setValue(50);
//            widget->layout()->addWidget(slider);
//            setpoint_slider_widget.push_back(slider);
//            QObject::connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setPointChangedSlider()));
//
//            motor_command_scrollarea->layout()->addWidget(widget);
//        }
//    }
//
//    setpoint_slider_widget.push_back(widget_->findChild<QSlider *>("motor_setPoint_slider_all"));
//    QObject::connect(setpoint_slider_widget.back(), SIGNAL(valueChanged(int)), this, SLOT(setPointAllChangedSlider()));
//    setpoint_widget.push_back(widget_->findChild<QLineEdit *>("motor_setPoint_all"));
//    QObject::connect(setpoint_widget.back(), SIGNAL(editingFinished()), this, SLOT(setPointAllChanged()));
//    scale = widget_->findChild<QLineEdit *>("motor_scale");
//
//    motorCommand = nh->advertise<roboy_communication_middleware::MotorCommand>("/roboy/middleware/MotorCommand", 1);
//    motorControl = nh->serviceClient<roboy_communication_middleware::ControlMode>("/roboy/middleware/ControlMode");
//    motorConfig = nh->serviceClient<roboy_communication_middleware::MotorConfigService>("/roboy/middleware/MotorConfig");
//    emergencyStop = nh->serviceClient<std_srvs::SetBool>("/roboy/middleware/EmergencyStop");
//
//    ui.stop_button_all->setStyleSheet("background-color: green");
//    QObject::connect(ui.stop_button_all, SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
//
////    for(uint motor = 0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++){
////        QObject::connect(setpoint_slider_widget.at(motor), SIGNAL(valueChanged(int)), this, SLOT(setPointChanged(int)));
////    }
////    QObject::connect(setpoint_slider_widget.at(NUMBER_OF_MOTORS_PER_FPGA), SIGNAL(valueChanged(int)), this, SLOT(setPointAllChanged(int)));
//
//    QObject::connect(ui.pos, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.vel, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.dis, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//    QObject::connect(ui.force, SIGNAL(clicked()), this, SLOT(controlModeChanged()));
//
//    QObject::connect(ui.update_config, SIGNAL(clicked()), this, SLOT(update_config()));
//
//    QObject::connect(ui.load_motor_config, SIGNAL(clicked()), this, SLOT(loadMotorConfig()));
//    loadMotorConfig();
}

void RoboyTrajectoriesControl::shutdownPlugin() {
    // TODO unregister all publishers here
}

void RoboyTrajectoriesControl::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyTrajectoriesControl::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyTrajectoriesControl::checkMotorStatus() {
    //TODO connect motor status listener
    QGraphicsScene* scene = new QGraphicsScene(this);
    QBrush greenBrush(Qt::green);

    QGraphicsView* view = widget_->findChild<QGraphicsView *>("motorStatus0");
    view->setScene(scene);
    view->setBackgroundBrush(greenBrush);
}

void RoboyTrajectoriesControl::motorStatusCallback(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    // TODO check if motor online
}

void RoboyTrajectoriesControl::pullExistingTrajectories() {

    // TODO ros service to get the list from FPGA
    roboy_communication_control::ListTrajectories srv;
    srv.request.folder = TRAJECTORIES_PATH;
    listExistingTrajectoriesServiceClient.call(srv);

    // empty list
    while(ui.existingTrajectories->count()>0)
    {
        ui.existingTrajectories->takeItem(0);
    }
    vector<QString> trajectories;
    for (string t: srv.response.trajectories) {
        trajectories.push_back(QString::fromStdString(t));
    }
    QStringList existingTrajectories = QStringList::fromVector(QVector<QString>::fromStdVector(trajectories));
    ui.existingTrajectories->addItems(existingTrajectories);

    connect(ui.existingTrajectories, SIGNAL(itemDoubleClicked(QListWidgetItem*)),
            this, SLOT(onExistingTrajectoriesItemClicked(QListWidgetItem*)));

}

void RoboyTrajectoriesControl::onExistingTrajectoriesItemClicked(QListWidgetItem* item) {
    QListWidgetItem* newItem = new QListWidgetItem();
    newItem->setWhatsThis("trajectory");
    newItem->setText(item->text());
    ui.scheduledBehavior->addItem(newItem);
    ui.clearBehavior->setEnabled(true);
    ui.playBehavior->setEnabled(true);
};

void RoboyTrajectoriesControl::refreshTrajectoriesButtonClicked() {
    pullExistingTrajectories();
}

void RoboyTrajectoriesControl::addPauseButtonClicked() {
    string pauseMsg(to_string(pauseDuration) + "s pause");
    QListWidgetItem * item = new QListWidgetItem();
    item->setText(QString::fromStdString(pauseMsg));
    item->setWhatsThis("pause");
    ui.scheduledBehavior->addItem(item);
    ui.pauseDuration->setValue(0);
    ui.clearBehavior->setEnabled(true);
    ui.playBehavior->setEnabled(true);
}

void RoboyTrajectoriesControl::setPauseDuration(int duration) {
    if (duration>0) {
        ui.addPause->setEnabled(true);
        pauseDuration = duration;
    } else {
        ui.addPause->setEnabled(false);
    }

}

void RoboyTrajectoriesControl::clearAllTrajectoriesButtonClicked() {
    while(ui.scheduledBehavior->count()>0)
    {
        ui.scheduledBehavior->takeItem(0);
    }
}

void RoboyTrajectoriesControl::playTrajectoriesButtonClicked() {
    vector<string> actions;
    for(int i = 0; i < ui.scheduledBehavior->count(); ++i)
    {
        QListWidgetItem* item = ui.scheduledBehavior->item(i);
        string actionName = item->text().toStdString();
        if (item->whatsThis().contains("trajectory")) {
            actions.push_back(actionName);
        }
        else if (item->whatsThis().contains("pause")) {
            string delimiter = "s";
            actions.push_back(actionName.substr(0, actionName.find(delimiter))+"_pause");
        }
    }

    roboy_communication_control::PerformBehavior srv;
    srv.request.actions = actions;
    executeBehaviorServiceClient.call(srv);

}

void RoboyTrajectoriesControl::relaxAllMusclesButtonClicked() {
    roboy_communication_middleware::SetInt16 srv;
    srv.request.setpoint = 0;
    setDisplacementForAllServiceClient.call(srv);
}

void RoboyTrajectoriesControl::startInitializationButtonClicked() {
    roboy_communication_middleware::SetInt16 srv;
    srv.request.setpoint = 50;
    setDisplacementForAllServiceClient.call(srv);
}

void RoboyTrajectoriesControl::startRecordTrajectoryButtonClicked() {
    roboy_communication_control::StartRecordTrajectory srv;

    if (ui.newTrajectoryName->toPlainText().isEmpty() || ui.newTrajectoryName->toPlainText().isNull()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No trajectory name specified");
        messageBox.setFixedSize(500,200);
    }
    else {

        srv.request.name = ui.newTrajectoryName->toPlainText().toStdString();
        // TODO provide selection of motors to record
        vector<int8_t> ids(total_number_of_motors);
        iota(begin(ids), end(ids), 0);
        srv.request.idList = ids;
        startRecordTrajectoryServiceClient.call(srv);
        ui.startRecord->setEnabled(false);
        ui.stopRecord->setEnabled(true);
    }
}

void RoboyTrajectoriesControl::stopRecordTrajectoryButtonClicked() {
    roboy_communication_control::StopRecordTrajectory srv;
    stopRecordTrajectoryServiceClient.call(srv);
    ui.startRecord->setEnabled(true);
    ui.stopRecord->setEnabled(false);
    ui.newTrajectoryName->clear();

}

//void RoboyTrajectoriesControl::stopButtonAllClicked(){
//    std_srvs::SetBool msg;
//    if(ui.stop_button_all->isChecked()) {
//        ui.stop_button_all->setStyleSheet("background-color: red");
//        msg.request.data = 1;
//        emergencyStop.call(msg);
//        ui.motor_command->setEnabled(false);
//        ui.pos->setEnabled(false);
//        ui.vel->setEnabled(false);
//        ui.dis->setEnabled(false);
//        ui.force->setEnabled(false);
//    }else {
//        ui.stop_button_all->setStyleSheet("background-color: green");
//        msg.request.data = 0;
//        emergencyStop.call(msg);
//        ui.motor_command->setEnabled(true);
//        ui.pos->setEnabled(true);
//        ui.vel->setEnabled(true);
//        ui.dis->setEnabled(true);
//        ui.force->setEnabled(true);
//    }
//}
//
//void RoboyTrajectoriesControl::setPointChanged(){
//    roboy_communication_middleware::MotorCommand msg;
//    bool ok;
//    double motor_scale = scale->text().toDouble(&ok);
//    if(!ok)
//        return;
//
//    for (uint motor = 0; motor < total_number_of_motors; motor++) {
//        double setPoint = setpoint_widget[motor]->text().toDouble(&ok) * motor_scale;
//        if(setpoint[motor] != setPoint && ok) {
//            setpoint[motor] = setPoint;
//            msg.id = motor / NUMBER_OF_MOTORS_PER_FPGA;
//            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
//            if (control_mode[motor] == FORCE) {
//                double displacement = force2displacement(setPoint, motor);
//                msg.setPoints.push_back(displacement);
//            } else {
//                msg.setPoints.push_back(setPoint);
//            }
//        }
//        setpoint_widget[motor]->setText(QString::number(setPoint));
//        if((motor+1)%NUMBER_OF_MOTORS_PER_FPGA==0){ // every fpga get his own message
//            if(msg.motors.size()>0)
//                motorCommand.publish(msg);
//            //clear the message for the next fpga
//            msg.motors.clear();
//            msg.setPoints.clear();
//        }
//    }
//}
//
//void RoboyTrajectoriesControl::setPointChangedSlider(){
//    roboy_communication_middleware::MotorCommand msg;
//    bool ok;
//    double motor_scale = scale->text().toDouble(&ok);
//    if(!ok){
//        ROS_ERROR("invalid scale");
//        return;
//    }
//
//    for (uint motor = 0; motor < total_number_of_motors; motor++) {
//        double setPoint = (setpoint_slider_widget[motor]->value()-50.0) * motor_scale;
//        if(setpoint[motor] != setPoint ) {
//            setpoint[motor] = setPoint;
//            msg.id = motor / NUMBER_OF_MOTORS_PER_FPGA;
//            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
//            if (control_mode[motor] == FORCE) {
//                double displacement = force2displacement(setPoint, motor);
//                msg.setPoints.push_back(displacement);
//            } else {
//                msg.setPoints.push_back(setPoint);
//            }
//        }
//        setpoint_widget[motor]->setText(QString::number(setPoint));
//        if((motor+1)%NUMBER_OF_MOTORS_PER_FPGA==0){ // every fpga get his own message
//            if(msg.motors.size()>0)
//                motorCommand.publish(msg);
//            //clear the message for the next fpga
//            msg.motors.clear();
//            msg.setPoints.clear();
//        }
//    }
//}
//
//void RoboyTrajectoriesControl::setPointAllChanged(){
//    roboy_communication_middleware::MotorCommand msg;
//    bool ok;
//    double motor_scale = scale->text().toDouble(&ok);
//    if(!ok){
//        ROS_ERROR("invalid scale");
//        return;
//    }
//
//    double setPoint = setpoint_widget.back()->text().toDouble(&ok) * motor_scale;
//    for (uint motor = 0; motor < total_number_of_motors; motor++) {
//        if(setpoint[motor] != setPoint) {
//            setpoint[motor] = setPoint;
//            msg.id = motor / NUMBER_OF_MOTORS_PER_FPGA;
//            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
//            if (control_mode[motor] == FORCE) {
//                double displacement = force2displacement(setPoint, motor);
//                msg.setPoints.push_back(displacement);
//            } else {
//                msg.setPoints.push_back(setPoint);
//            }
//        }
//        setpoint_widget[motor]->setText(QString::number(setPoint));
//        if((motor+1)%NUMBER_OF_MOTORS_PER_FPGA==0){ // every fpga get his own message
//            if(msg.motors.size()>0)
//                motorCommand.publish(msg);
//            //clear the message for the next fpga
//            msg.motors.clear();
//            msg.setPoints.clear();
//        }
//    }
//}
//
//void RoboyTrajectoriesControl::setPointAllChangedSlider(){
//    roboy_communication_middleware::MotorCommand msg;
//    bool ok;
//    double motor_scale = scale->text().toDouble(&ok);
//    if(!ok){
//        ROS_ERROR("invalid scale");
//        return;
//    }
//
//    double setPoint = (setpoint_slider_widget.back()->value()-50.0) * motor_scale;
//    ui.motor_setPoint_all->setText(QString::number(setPoint));
//    for (uint motor = 0; motor < total_number_of_motors; motor++) {
//        if(setpoint[motor] != setPoint) {
//            setpoint[motor] = setPoint;
//            msg.id = motor / NUMBER_OF_MOTORS_PER_FPGA;
//            msg.motors.push_back(motor % NUMBER_OF_MOTORS_PER_FPGA);
//            if (control_mode[motor] == FORCE) {
//                double displacement = force2displacement(setPoint, motor);
//                msg.setPoints.push_back(displacement);
//            } else {
//                msg.setPoints.push_back(setPoint);
//            }
//        }
//        setpoint_widget[motor]->setText(QString::number(setPoint));
//        if((motor+1)%NUMBER_OF_MOTORS_PER_FPGA==0){ // every fpga get his own message
//            if(msg.motors.size()>0)
//                motorCommand.publish(msg);
//            //clear the message for the next fpga
//            msg.motors.clear();
//            msg.setPoints.clear();
//        }
//    }
//}
//
//void RoboyTrajectoriesControl::controlModeChanged(){
//    roboy_communication_middleware::ControlMode msg;
//    if(ui.pos->isChecked()) {
//        for(int motor = 0; motor<total_number_of_motors; motor++){
//            control_mode[motor] = POSITION;
//            pos[motor]->setChecked(true);
//        }
//        msg.request.control_mode = POSITION;
//    }
//    if(ui.vel->isChecked()) {
//        for(int motor = 0; motor<total_number_of_motors; motor++){
//            control_mode[motor] = VELOCITY;
//            vel[motor]->setChecked(true);
//        }
//        msg.request.control_mode = VELOCITY;
//    }
//    if(ui.dis->isChecked()) {
//        for(int motor = 0; motor<total_number_of_motors; motor++){
//            control_mode[motor] = DISPLACEMENT;
//            dis[motor]->setChecked(true);
//        }
//        msg.request.control_mode = DISPLACEMENT;
//    }
//    if(ui.force->isChecked()) {
//        for(int motor = 0; motor<total_number_of_motors; motor++){
//            control_mode[motor] = FORCE;
//            force[motor]->setChecked(true);
//        }
//        msg.request.control_mode = DISPLACEMENT;
//    }
//
//    bool ok;
//    double motor_scale = scale->text().toDouble(&ok);
//    if(!ok){
//        ROS_ERROR("invalid scale");
//        return;
//    }
//    msg.request.setPoint = setpoint_slider_widget.back()->value() * motor_scale;
//    if(!motorControl.call(msg))
//        ROS_ERROR("failed to change control mode, is emergency stop active?! are the fpgas connected?!");
//}
//
//void RoboyTrajectoriesControl::update_config(){
//    bool ok;
//    int outputPosMax = ui.outputPosMax->text().toInt(&ok);
//    if(!ok || outputPosMax>4000) {
//        ROS_ERROR("outputPosMax not valid [0 to 4000]");
//        return;
//    }
//    int outputNegMax = ui.outputNegMax->text().toInt(&ok);
//    if(!ok || outputNegMax<-4000) {
//        ROS_ERROR("outputNegMax not valid [0 to -4000]");
//        return;
//    }
//    int spPosMax = ui.spNegMax->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("spPosMax not valid");
//        return;
//    }
//    int spNegMax = ui.spNegMax->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("spNegMax not valid");
//        return;
//    }
//    int integralPosMax = ui.integralPosMax->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("integralPosMax not valid");
//        return;
//    }
//    int integralNegMax = ui.integralNegMax->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("integralNegMax not valid");
//        return;
//    }
//    int deadband = ui.deadband->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("deadband not valid");
//        return;
//    }
//    int Kp = ui.Kp->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("Kp not valid");
//        return;
//    }
//    int Ki = ui.Ki->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("Ki not valid");
//        return;
//    }
//    int Kd = ui.Kd->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("Kd not valid");
//        return;
//    }
//    int forwardGain = ui.forwardGain->text().toInt(&ok);
//    if(!ok) {
//        ROS_ERROR("forwardGain not valid");
//        return;
//    }
//    roboy_communication_middleware::MotorConfigService msg;
//    for(int i=0;i<NUMBER_OF_MOTORS_PER_FPGA;i++){
//        msg.request.config.motors.push_back(i);
//        msg.request.config.control_mode.push_back(control_mode[i]);
//        msg.request.config.outputPosMax.push_back(outputPosMax);
//        msg.request.config.outputNegMax.push_back(outputNegMax);
//        msg.request.config.spPosMax.push_back(spPosMax);
//        msg.request.config.spNegMax.push_back(spNegMax);
//        msg.request.config.IntegralPosMax.push_back(integralPosMax);
//        msg.request.config.IntegralNegMax.push_back(integralNegMax);
//        msg.request.config.deadBand.push_back(deadband);
//        msg.request.config.Kp.push_back(Kp);
//        msg.request.config.Ki.push_back(Ki);
//        msg.request.config.Kd.push_back(Kd);
//        msg.request.config.forwardGain.push_back(forwardGain);
//        msg.request.setPoints.push_back(setpoint[i]);
//    }
//    motorConfig.call(msg);
//}
//
//void RoboyTrajectoriesControl::loadMotorConfig(){
//    readConfig(ui.motor_config_path->text().toStdString());
//}

PLUGINLIB_DECLARE_CLASS(roboy_trajectories_control, RoboyTrajectoriesControl, RoboyTrajectoriesControl, rqt_gui_cpp::Plugin)
