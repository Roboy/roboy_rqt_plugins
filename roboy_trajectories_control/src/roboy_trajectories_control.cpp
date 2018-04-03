#include <roboy_trajectories_control/roboy_trajectories_control.hpp>


RoboyTrajectoriesControl::RoboyTrajectoriesControl()
        : rqt_gui_cpp::Plugin(),
          widget_(0),
          performMovements_ac("movements_server", true),
          performMovement_ac("movement_server", true)
           {
               setObjectName("RoboyTrajectoriesControl");
//               ROS_INFO("Waiting for action server to start.");
               // wait for the action server to start
               ros::Duration timeout(2);
               performMovements_ac.waitForServer(timeout);
               performMovement_ac.waitForServer(timeout);
               if (!performMovement_ac.isServerConnected() || !performMovements_ac.isServerConnected())
               {
                   ROS_WARN("Could not connect to the action server. Movements might not be available");
               }
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

    connect(ui.clearBehavior, SIGNAL(clicked()), this, SLOT(clearAllTrajectoriesButtonClicked()));
    connect(ui.playBehavior, SIGNAL(clicked()), this, SLOT(playTrajectoriesButtonClicked()));
    connect(ui.stopBehavior, SIGNAL(clicked()), this, SLOT(stopBehaviorButtonClicked()));
    connect(ui.refreshTrajectories, SIGNAL(clicked()), this, SLOT(refreshTrajectoriesButtonClicked()));
    connect(ui.addPause, SIGNAL(clicked()), this, SLOT(addPauseButtonClicked()));
    connect(ui.addRelax, SIGNAL(clicked()), this, SLOT(addRelaxButtonClicked()));
    connect(ui.relaxAll, SIGNAL(clicked()), this, SLOT(relaxAllMusclesButtonClicked()));
    connect(ui.startInit, SIGNAL(clicked()), this, SLOT(startInitializationButtonClicked()));
    connect(ui.startRecord, SIGNAL(clicked()), this, SLOT(startRecordTrajectoryButtonClicked()));
    connect(ui.stopRecord, SIGNAL(clicked()), this, SLOT(stopRecordTrajectoryButtonClicked()));
    connect(ui.saveBehavior, SIGNAL(clicked()), this, SLOT(saveBehaviorButtonClicked()));
    connect(ui.loadBehavior, SIGNAL(clicked()), this, SLOT(loadBehaviorButtonClicked()));

    connect(ui.pauseDuration, SIGNAL(valueChanged(int)), this, SLOT(setPauseDuration(int)));
    connect(ui.timeUnits, SIGNAL(currentIndexChanged(int)), this, SLOT(setTimeUnits(int)));

    connect(ui.existingTrajectories, SIGNAL(itemClicked(QListWidgetItem*)),
            this, SLOT(onExistingTrajectoriesItemClicked(QListWidgetItem*)));
    connect(ui.scheduledBehavior, SIGNAL(itemClicked(QListWidgetItem*)),
            this, SLOT(onScheduledBehaviorItemClicked(QListWidgetItem*)));

    ui.playBehavior->setEnabled(false);
    ui.clearBehavior->setEnabled(false);
    ui.addPause->setEnabled(false);
    ui.stopRecord->setEnabled(false);
    ui.saveBehavior->setEnabled(false);
    ui.stopBehavior->setEnabled(false);
    ui.timeUnits->addItem("secs");
    ui.timeUnits->addItem("mins");
    ui.timeUnits->addItem("hours");
    ui.timeUnits->addItem("days");

    // TODO get rid of this!!
    motorStatusViews.push_back(ui.motorStatus0);
    motorStatusViews.push_back(ui.motorStatus1);
    motorStatusViews.push_back(ui.motorStatus2);
    motorStatusViews.push_back(ui.motorStatus3);
    motorStatusViews.push_back(ui.motorStatus4);
    motorStatusViews.push_back(ui.motorStatus5);
    for (auto view: motorStatusViews) {
        view->setScene(new QGraphicsScene(this));
    }


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
    executeActionsServiceClient = nh->serviceClient<roboy_communication_control::PerformActions>("/roboy/control/ExecuteActions");
    listExistingTrajectoriesServiceClient = nh->serviceClient<roboy_communication_control::ListItems>("roboy/control/ListExistingTrajectories");
    listExistingBehaviorsServiceClient = nh->serviceClient<roboy_communication_control::ListItems>("roboy/control/ListExistingTrajectories");
    expandBehaviorServiceClient = nh->serviceClient<roboy_communication_control::ListItems>("roboy/control/ExpandBehavior");

    motorStatusSubscriber = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyTrajectoriesControl::motorStatusCallback, this);

    startRecordTrajectoryPublisher = nh->advertise<roboy_communication_control::StartRecordTrajectory>("/roboy/control/StartRecordTrajectory", 1);
    stopRecordTrajectoryPublisher = nh->advertise<std_msgs::Empty>("/roboy/control/StopRecordTrajectory", 1);
    saveBehaviorPublisher = nh->advertise<roboy_communication_control::Behavior>("/roboy/control/SaveBehavior", 1);
    enablePlaybackPublisher = nh->advertise<std_msgs::Bool>("/roboy/control/EnablePlayback", 1);

//    if (!performMovements_ac.isServerConnected()) {
//        ROS_ERROR("perform movements action server does not exist");
//    }
//    if (!performMovement_ac.isServerConnected()) {
//        ROS_ERROR("perform movement action server does not exist");
//    }

    // TODO wait for existence of services?

//    motorCommandPublisher = nh->advertise<roboy_communication_middleware::motorCommand>("/roboy/middleware/MotorCommand", 1);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    pullExistingTrajectories();
//    Server server(*nh, "do_dishes", boost::bind(&execute, _1, &server), false);

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

void RoboyTrajectoriesControl::motorStatusCallback(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {

    QBrush greenBrush(Qt::green);
    QBrush redBrush(Qt::red);

    for (int i=0; i<total_number_of_motors; i++)
    {
        if (msg->current.at(i) > 0) {
            motorStatusViews.at(i)->setBackgroundBrush(greenBrush);
        }
        else {
            motorStatusViews.at(i)->setBackgroundBrush(redBrush);
        }
    }
}

void RoboyTrajectoriesControl::pullExistingTrajectories() {

    roboy_communication_control::ListItems srv;
    srv.request.name = trajectories_path;
    listExistingTrajectoriesServiceClient.call(srv);

    // empty list
    while(ui.existingTrajectories->count()>0)
    {
        ui.existingTrajectories->takeItem(0);
    }
    vector<QString> trajectories;
    for (string t: srv.response.items) {
        trajectories.push_back(QString::fromStdString(t));
    }
    QStringList existingTrajectories = QStringList::fromVector(QVector<QString>::fromStdVector(trajectories));
    ui.existingTrajectories->addItems(existingTrajectories);

}

void RoboyTrajectoriesControl::pullExistingBehaviors() {

    QStringList q;
    q.push_back("1on");
    q.push_back("wet");
    QInputDialog *dialog = new QInputDialog();
    bool accepted;
    QString item = dialog->getItem(0, "Title", "Label:", q, 0, false, &accepted);

    if (accepted && !item.isEmpty()) {
        vector<string> trajectories = expandBehavior(item.toStdString());

    }

//    roboy_communication_control::ListItems srv;
//    srv.request.folder = behaviors_path;
//    listExistingTrajectoriesServiceClient.call(srv);
//
//    // empty list
//    while(ui.existingTrajectories->count()>0)
//    {
//        ui.existingTrajectories->takeItem(0);
//    }
//    vector<QString> trajectories;
//    for (string t: srv.response.trajectories) {
//        trajectories.push_back(QString::fromStdString(t));
//    }
//    QStringList existingTrajectories = QStringList::fromVector(QVector<QString>::fromStdVector(trajectories));
//    ui.existingTrajectories->addItems(existingTrajectories);

}


void RoboyTrajectoriesControl::onExistingTrajectoriesItemClicked(QListWidgetItem* item) {
    QListWidgetItem* newItem = new QListWidgetItem();
    newItem->setWhatsThis("trajectory");
    newItem->setText(item->text());
    ui.scheduledBehavior->addItem(newItem);
    ui.clearBehavior->setEnabled(true);
    ui.playBehavior->setEnabled(true);
    ui.saveBehavior->setEnabled(true);
};

void RoboyTrajectoriesControl::onScheduledBehaviorItemClicked(QListWidgetItem* item) {
    ui.scheduledBehavior->takeItem(ui.scheduledBehavior->row(item));
    if (ui.scheduledBehavior->count()==0) {
        ui.clearBehavior->setEnabled(false);
        ui.playBehavior->setEnabled(false);
        ui.saveBehavior->setEnabled(false);
    }
};

void RoboyTrajectoriesControl::refreshTrajectoriesButtonClicked() {
    pullExistingTrajectories();
}

void RoboyTrajectoriesControl::addPauseButtonClicked() {
    string pauseMsg(to_string(timeFactor*pauseDuration) + "s pause");
    QListWidgetItem * item = new QListWidgetItem();
    item->setText(QString::fromStdString(pauseMsg));
    item->setWhatsThis("pause");
    ui.scheduledBehavior->addItem(item);
    ui.pauseDuration->setValue(0);
    ui.clearBehavior->setEnabled(true);
    ui.playBehavior->setEnabled(true);
    ui.saveBehavior->setEnabled(true);
}

void RoboyTrajectoriesControl::addRelaxButtonClicked() {
    QListWidgetItem * item = new QListWidgetItem();
    item->setText("relax muscles");
    item->setWhatsThis("relax");
    ui.scheduledBehavior->addItem(item);
    ui.clearBehavior->setEnabled(true);
    ui.playBehavior->setEnabled(true);
}

void RoboyTrajectoriesControl::setTimeUnits(int idx) {
    if (ui.timeUnits->currentText().contains("secs")) {
        timeFactor = 1;
    }
    if (ui.timeUnits->currentText().contains("mins")) {
        timeFactor = 60;
    }
    else if (ui.timeUnits->currentText().contains("hours")) {
        timeFactor = 3600;
    }
    else if (ui.timeUnits->currentText().contains("days")) {
        timeFactor = 24*3600;
    }
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
    ui.saveBehavior->setEnabled(false);
    ui.playBehavior->setEnabled(false);
    ui.clearBehavior->setEnabled(false);
    ui.stopBehavior->setEnabled(false);
}

void RoboyTrajectoriesControl::playTrajectoriesButtonClicked() {
    std_msgs::Bool msg;
    msg.data = true;
    enablePlaybackPublisher.publish(msg);

    vector<string> actions = getCurrentActions();

    roboy_communication_control::PerformMovementsGoal goal;
    goal.actions = actions;
    performMovements_ac.sendGoal(goal);
    ui.stopBehavior->setEnabled(true);

}

void RoboyTrajectoriesControl::stopBehaviorButtonClicked() {
    std_msgs::Bool msg;
    msg.data = false;
    enablePlaybackPublisher.publish(msg);

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
        replace( msg.name.begin(), msg.name.end(), ' ', '_');
        msg.name.erase(std::remove(msg.name.begin(), msg.name.end(), ','), msg.name.end());
        // TODO provide selection of motors to record
        vector<int8_t> ids(total_number_of_motors);
        iota(begin(ids), end(ids), 0);
        msg.idList = ids;
        startRecordTrajectoryPublisher.publish(msg);
        ui.startRecord->setEnabled(false);
        ui.stopRecord->setEnabled(true);
    }
}

void RoboyTrajectoriesControl::saveBehaviorButtonClicked() {

    roboy_communication_control::Behavior msg;
    bool ok;
    msg.name = (QInputDialog::getText(0, "Save behavior as...",
                                         "Name:", QLineEdit::Normal,
                                         "", &ok)).toStdString();
    if (ok && msg.name.length()!=0) {
        replace( msg.name.begin(), msg.name.end(), ' ', '_');
        msg.name.erase(std::remove(msg.name.begin(), msg.name.end(), ','), msg.name.end());
    } else {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","No behavior name specified");
        messageBox.setFixedSize(500,200);
        return;
    }

    msg.actions = getCurrentActions();

    saveBehaviorPublisher.publish(msg);
}

void RoboyTrajectoriesControl::loadBehaviorButtonClicked() {

    roboy_communication_control::ListItems srv;
    srv.request.name = behaviors_path;
    listExistingBehaviorsServiceClient.call(srv);

    vector<QString> trajectories;
    for (string t: srv.response.items) {
        trajectories.push_back(QString::fromStdString(t));
    }
    QStringList existingBehaviors = QStringList::fromVector(QVector<QString>::fromStdVector(trajectories));
    QInputDialog *dialog = new QInputDialog();
    bool accepted;
    QString item = dialog->getItem(0, "Load behavior...", "Behavior:", existingBehaviors, 0, false, &accepted);

    if (accepted && !item.isEmpty()) {
        vector<string> actions = expandBehavior(item.toStdString());
        while(ui.scheduledBehavior->count()>0)
        {
            ui.scheduledBehavior->takeItem(0);
        }
        for (string action: actions) {
            QListWidgetItem* item = new QListWidgetItem();
            item->setText(QString::fromStdString(action));
            if (action.find("pause") != std::string::npos) {
                item->setWhatsThis("pause");
                item->setText(QString::fromStdString(action.substr(0, action.find("_"))+"s pause"));
                ROS_INFO_STREAM(action.substr(0, action.find("_"))+"s pause");
            }
            else if (action.find("relax") != std::string::npos) {
                item->setWhatsThis("relax");
                item->setText("relax");
            }
            else {
                item->setWhatsThis("trajectory");
                item->setText(QString::fromStdString(action));
            }
            //TODO investigate why previously added (before load) items vanish
            ui.scheduledBehavior->addItem(item);
        }

        ui.saveBehavior->setEnabled(true);
        ui.playBehavior->setEnabled(true);
        ui.clearBehavior->setEnabled(true);
    }


}

void RoboyTrajectoriesControl::stopRecordTrajectoryButtonClicked() {
    std_msgs::Empty msg;
    stopRecordTrajectoryPublisher.publish(msg);
    ui.startRecord->setEnabled(true);
    ui.stopRecord->setEnabled(false);
    ui.newTrajectoryName->clear();

}

vector<string> RoboyTrajectoriesControl::getCurrentActions() {
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
        else if (item->whatsThis().contains("relax")) {
            actions.push_back("relax");
        }

    }

    return actions;
}

vector<string> RoboyTrajectoriesControl::expandBehavior(string name) {
    roboy_communication_control::ListItems srv;
    srv.request.name = name;
    expandBehaviorServiceClient.call(srv);

    return srv.response.items;

}

PLUGINLIB_DECLARE_CLASS(roboy_trajectories_control, RoboyTrajectoriesControl, RoboyTrajectoriesControl, rqt_gui_cpp::Plugin)
