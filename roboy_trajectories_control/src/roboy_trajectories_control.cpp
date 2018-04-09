#include <roboy_trajectories_control/roboy_trajectories_control.hpp>
#include <dirent.h>


RoboyTrajectoriesControl::RoboyTrajectoriesControl()
        : rqt_gui_cpp::Plugin(),
          widget_(0),
//          motorStatus(bodyParts.size()),
          greenBrush(Qt::green),
          redBrush(Qt::red)
           {
               setObjectName("RoboyTrajectoriesControl");
//               ROS_INFO("Waiting for action server to start.");
               // wait for the action server to start
                ros::Duration timeout(0.2);

                for (auto part: bodyParts)
                {
                    performMovements_ac[part] = new MovementsAC(part+"_movements_server", true);
                }

                for (auto ac: performMovement_ac) {
                    ac.second->waitForServer(timeout);
                    if (!ac.second->isServerConnected()) {
                        ROS_WARN_STREAM("Could not connect to the action server " + ac.first + ". Movements might not be available");
                    }
                }

                for (auto ac: performMovements_ac) {
                    ac.second->waitForServer(timeout);
                    if (!ac.second->isServerConnected()) {
                        ROS_WARN_STREAM("Could not connect to the action server " + ac.first + ". Movements might not be available");
                    }
                }

                total_number_of_motors.reserve(bodyParts.size());
                total_number_of_motors[HEAD] = 4;
                total_number_of_motors[SHOULDER_LEFT] = 13;
                total_number_of_motors[SHOULDER_RIGHT] = 13;
                total_number_of_motors[SPINE_LEFT] = 3;
                total_number_of_motors[SPINE_RIGHT] = 3;
                total_number_of_motors[LEGS] = 12;



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

    scene = new QGraphicsScene(widget_);
    scene->setBackgroundBrush(redBrush);

    connect(ui.clearBehavior, SIGNAL(clicked()), this, SLOT(clearAllTrajectoriesButtonClicked()));
    connect(ui.playBehavior, SIGNAL(clicked()), this, SLOT(playTrajectoriesButtonClicked()));
    connect(ui.stopBehavior, SIGNAL(clicked()), this, SLOT(stopBehaviorButtonClicked()));
    connect(ui.refreshTrajectories, SIGNAL(clicked()), this, SLOT(refreshTrajectoriesButtonClicked()));
    connect(ui.addPause, SIGNAL(clicked()), this, SLOT(addPauseButtonClicked()));
    connect(ui.addRelax, SIGNAL(clicked()), this, SLOT(addRelaxButtonClicked()));
    connect(ui.addSync, SIGNAL(clicked()), this, SLOT(addSyncButtonClicked()));
    connect(ui.relaxAll, SIGNAL(clicked()), this, SLOT(relaxAllMusclesButtonClicked()));
    connect(ui.startInit, SIGNAL(clicked()), this, SLOT(startInitializationButtonClicked()));
    connect(ui.startRecord, SIGNAL(clicked()), this, SLOT(startRecordTrajectoryButtonClicked()));
    connect(ui.stopRecord, SIGNAL(clicked()), this, SLOT(stopRecordTrajectoryButtonClicked()));
    connect(ui.saveBehavior, SIGNAL(clicked()), this, SLOT(saveBehaviorButtonClicked()));
    connect(ui.loadBehavior, SIGNAL(clicked()), this, SLOT(loadBehaviorButtonClicked()));
    connect(ui.setPredisplacement,SIGNAL(clicked()), this, SLOT(setPredisplacementButtonClicked()));

    connect(ui.pauseDuration, SIGNAL(valueChanged(int)), this, SLOT(setPauseDuration(int)));
    connect(ui.predisplacement, SIGNAL(valueChanged(int)), this, SLOT(setPredisplacement(int)));
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

    ui.progressBar->hide();

    for (int i=0; i<bodyParts.size(); i++) {

        activeBodyParts.push_back(widget_->findChild<QCheckBox*>(QString::fromStdString(bodyParts[i])));
        activeBodyParts[i]->setChecked(true);

        motorStatus.push_back(vector<bool>(total_number_of_motors[i]));
        for (int j=0; j<total_number_of_motors[i]; j++) {
            QGraphicsView* view = widget_->findChild<QGraphicsView *>(QString::fromStdString(bodyParts[i]+"_m"+to_string(j)));
//            ROS_INFO_STREAM(view->whatsThis().toStdString());

            view->setScene(scene);
//            view->setBackgroundBrush(redBrush);
//            ROS_INFO_STREAM(view->objectName().toStdString());
            motorStatus[i].push_back(false);
        }
    }

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "trajectories_control_rqt_plugin");
    }

    initializeRosCommunication();
    ros::AsyncSpinner spinner(0);
    spinner.start();

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

void RoboyTrajectoriesControl::initializeRosCommunication() {

    for (auto body_part: bodyParts) {
        emergencyStopServiceClient[body_part] = nh->serviceClient<std_srvs::SetBool>("/roboy/" + body_part + "/middleware/EmergencyStop");
        setDisplacementForAllServiceClient[body_part] = nh->serviceClient<roboy_communication_middleware::SetInt16>("/roboy/" + body_part + "/middleware/SetDisplacementForAll");
        performMovementServiceClient[body_part] = nh->serviceClient<roboy_communication_control::PerformMovement>("/roboy/" + body_part + "/control/ReplayTrajectory");
        executeActionsServiceClient[body_part] = nh->serviceClient<roboy_communication_control::PerformActions>("/roboy/" + body_part + "/control/ExecuteActions");
        listExistingTrajectoriesServiceClient[body_part] = nh->serviceClient<roboy_communication_control::ListItems>("/roboy/" + body_part + "/control/ListExistingTrajectories");

        performMovementsResultSubscriber[body_part] = nh->subscribe("/"+body_part+"_movements_server/result", 1, &RoboyTrajectoriesControl::performMovementsResultCallback, this);
    }

//    listExistingBehaviorsServiceClient = nh->serviceClient<roboy_communication_control::ListItems>("/roboy/control/ListExistingTrajectories");
//    expandBehaviorServiceClient = nh->serviceClient<roboy_communication_control::ListItems>("/roboy/control/ExpandBehavior");
    motorStatusSubscriber = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyTrajectoriesControl::motorStatusCallback, this);

    startRecordTrajectoryPublisher = nh->advertise<roboy_communication_control::StartRecordTrajectory>("/roboy/control/StartRecordTrajectory", 1);
    stopRecordTrajectoryPublisher = nh->advertise<std_msgs::Empty>("/roboy/control/StopRecordTrajectory", 1);
//    saveBehaviorPublisher = nh->advertise<roboy_communication_control::Behavior>("/roboy/control/SaveBehavior", 1);
    enablePlaybackPublisher = nh->advertise<std_msgs::Bool>("/roboy/control/EnablePlayback", 1);
    preDisplacementPublisher = nh->advertise<std_msgs::Int32>("/roboy/middleware/PreDisplacement", 1);

    // TODO warn if service doesnt exist

}

void RoboyTrajectoriesControl::performMovementsResultCallback(const roboy_communication_control::PerformMovementsActionResult::ConstPtr &msg) {
    // TODO many parts with various duration time
    ui.progressBar->hide();
}

void RoboyTrajectoriesControl::motorStatusCallback(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {

    for (int i=0; i<total_number_of_motors[msg->id]; i++)
    {
        if (msg->current.at(i) > 0 && !motorStatus[msg->id][i]) {
            widget_->findChild<QGraphicsView *>(
                    QString::fromStdString(bodyParts[msg->id]+"_m"+to_string(i)))->setBackgroundBrush(greenBrush);
            motorStatus[msg->id][i] = true;
        }
        else if (msg->current.at(i) < 0 && motorStatus[msg->id][i]){
            widget_->findChild<QGraphicsView *>(
                    QString::fromStdString(bodyParts[msg->id]+"_m"+to_string(i)))->setBackgroundBrush(redBrush);
            motorStatus[msg->id][i] = false;
        }
    }
}

void RoboyTrajectoriesControl::pullExistingTrajectories() {

    vector<QString> trajectories;

    for (auto part: bodyParts) {
        roboy_communication_control::ListItems srv;
        srv.request.name = trajectories_path;
        listExistingTrajectoriesServiceClient[part].call(srv);

        for (string t: srv.response.items) {
            trajectories.push_back(QString::fromStdString(t));
        }
    }

    // empty list
    while(ui.existingTrajectories->count()>0)
    {
        ui.existingTrajectories->takeItem(0);
    }

    QStringList existingTrajectories = QStringList::fromVector(QVector<QString>::fromStdVector(trajectories));
    existingTrajectories.sort();
    ui.existingTrajectories->addItems(existingTrajectories);

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

void RoboyTrajectoriesControl::addSyncButtonClicked() {
    QListWidgetItem * item = new QListWidgetItem();
    item->setText("sync body parts");
    item->setWhatsThis("sync");
    ui.scheduledBehavior->addItem(item);
    ui.clearBehavior->setEnabled(true);
//    ui.playBehavior->setEnabled(true);
}

void RoboyTrajectoriesControl::setPredisplacementButtonClicked() {
    std_msgs::Int32 msg;
    msg.data = preDisplacement;
    preDisplacementPublisher.publish(msg);
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

void RoboyTrajectoriesControl::setPredisplacement(int value) {
    if (value>0) {
        ui.setPredisplacement->setEnabled(true);
        preDisplacement = value;
    } else {
        ui.setPredisplacement->setEnabled(false);
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

    addSyncButtonClicked();
    ui.progressBar->show();
    ui.progressBar->setMaximum(0);
    ui.progressBar->setMinimum(0);
    std_msgs::Bool msg;
    msg.data = true;
    enablePlaybackPublisher.publish(msg);

    vector<string> actions = getCurrentActions();
    map<string,roboy_communication_control::PerformMovementsGoal> goals;
    for (auto action: actions) {
        if (action=="sync") {

            //TODO sync here
            for (auto ac: performMovements_ac)
            {
                auto currentGoal = goals[ac.first];
                if (currentGoal.actions.size() != 0) {
                    ROS_INFO_STREAM(ac.first);
                    ROS_INFO_STREAM(currentGoal.actions[0]);
                    ac.second->sendGoal(goals[ac.first]);
                    goals[ac.first].actions = {};
                }
            }

//            for (auto ac: performMovements_ac)
//            {
//                ac.second->waitForResult();
//            }

        }
        else {
            for (auto part: bodyParts) {
                // send only the relevant body parts to the corresponding actions servers
                if (action.find(part) != string::npos ||
                    action.find("relax") != string::npos ||
                    action.find("pause") != string::npos)
                    {
                        goals[part].actions.push_back(action);
                    }
            }
        }
    }

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
    for (auto part: bodyParts) {
        setDisplacementForAllServiceClient[part].call(srv);
    }
}

void RoboyTrajectoriesControl::startInitializationButtonClicked() {
    roboy_communication_middleware::SetInt16 srv;
    srv.request.setpoint = 30;
    for (auto part: bodyParts) {
        setDisplacementForAllServiceClient[part].call(srv);
    }
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
        vector<int8_t> ids(total_number_of_motors[0]);
        iota(begin(ids), end(ids), 0);
        msg.idList = ids;
        for (auto part: activeBodyParts) {
            if (part->isChecked()) {
                msg.body_parts.push_back(part->whatsThis().toStdString());
            }
        }
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

//    roboy_communication_control::ListItems srv;
//    srv.request.name = behaviors_path;
//    listExistingBehaviorsServiceClient.call(srv);


    vector<QString> trajectories;
    for (string t: listExistingBehaviors()) {
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
//                ROS_INFO_STREAM(action.substr(0, action.find("_"))+"s pause");
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
    pullExistingTrajectories();

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
        else if  (item->whatsThis().contains("sync")) {
            actions.push_back("sync");
        }
        else if (item->whatsThis().contains("relax")) {
            actions.push_back("relax");
        }
        else {
            ROS_WARN_STREAM("Unknown action in the list: " + actionName );
        }

    }

    return actions;
}

vector<string> RoboyTrajectoriesControl::listExistingBehaviors() {
    vector<string> res;
    DIR* dirp = opendir(behaviors_path.c_str());
    struct dirent * dp;
    if (!dirp) {
        mkdir(behaviors_path.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
        dirp = opendir(behaviors_path.c_str());
    }
    while ((dp = readdir(dirp)) != NULL) {
        if(dp->d_type!=DT_DIR) {
            res.push_back(dp->d_name);
        }
    }
    closedir(dirp);
    return res;
}

vector<string> RoboyTrajectoriesControl::expandBehavior(string name) {

    std::vector<string> actions;
    ifstream input_file(behaviors_path+name);
    std::copy(std::istream_iterator<std::string>(input_file),
              std::istream_iterator<std::string>(),
              std::back_inserter(actions));

    return actions;

//    roboy_communication_control::ListItems srv;
//    srv.request.name = name;
//    expandBehaviorServiceClient.call(srv);
//
//    return srv.response.items;

}

PLUGINLIB_DECLARE_CLASS(roboy_trajectories_control, RoboyTrajectoriesControl, RoboyTrajectoriesControl, rqt_gui_cpp::Plugin)
