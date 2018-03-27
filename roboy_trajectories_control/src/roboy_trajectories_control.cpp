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
    performMovementServiceClient = nh->serviceClient<roboy_communication_control::PerformMovement>("/roboy/control/ReplayTrajectory");
    executeBehaviorServiceClient = nh->serviceClient<roboy_communication_control::PerformBehavior>("/roboy/control/ExecuteBehavior");
    listExistingTrajectoriesServiceClient = nh->serviceClient<roboy_communication_control::ListTrajectories>("roboy/control/ListExistingTrajectories");

    motorStatusSubscriber = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyTrajectoriesControl::motorStatusCallback, this);

    startRecordTrajectoryPublisher = nh->advertise<roboy_communication_control::StartRecordTrajectory>("/roboy/control/StartRecordTrajectory", 1);
    stopRecordTrajectoryPublisher = nh->advertise<std_msgs::Empty>("/roboy/control/StopRecordTrajectory", 1);
//    motorCommandPublisher = nh->advertise<roboy_communication_middleware::motorCommand>("/roboy/middleware/MotorCommand", 1);

    checkMotorStatus();
    pullExistingTrajectories();

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

    connect(ui.existingTrajectories, SIGNAL(itemClicked(QListWidgetItem*)),
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
    roboy_communication_control::StartRecordTrajectory msg;



    if (ui.newTrajectoryName->toPlainText().isEmpty() || ui.newTrajectoryName->toPlainText().isNull()) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No trajectory name specified");
        messageBox.setFixedSize(500,200);
    }
    else {

        msg.name = ui.newTrajectoryName->toPlainText().toStdString();
        // TODO provide selection of motors to record
        vector<int8_t> ids(total_number_of_motors);
        iota(begin(ids), end(ids), 0);
        msg.idList = ids;
        startRecordTrajectoryPublisher.publish(msg);
        ui.startRecord->setEnabled(false);
        ui.stopRecord->setEnabled(true);
    }
}

void RoboyTrajectoriesControl::stopRecordTrajectoryButtonClicked() {
    std_msgs::Empty msg;
    stopRecordTrajectoryPublisher.publish(msg);
    ui.startRecord->setEnabled(true);
    ui.stopRecord->setEnabled(false);
    ui.newTrajectoryName->clear();

}

PLUGINLIB_DECLARE_CLASS(roboy_trajectories_control, RoboyTrajectoriesControl, RoboyTrajectoriesControl, rqt_gui_cpp::Plugin)
