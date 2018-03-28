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



    connect(ui.clearBehavior, SIGNAL(clicked()), this, SLOT(clearAllTrajectoriesButtonClicked()));
    connect(ui.playBehavior, SIGNAL(clicked()), this, SLOT(playTrajectoriesButtonClicked()));
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

    connect(ui.existingTrajectories, SIGNAL(itemClicked(QListWidgetItem*)),
            this, SLOT(onExistingTrajectoriesItemClicked(QListWidgetItem*)));
    connect(ui.scheduledBehavior, SIGNAL(itemClicked(QListWidgetItem*)),
            this, SLOT(onScheduledBehaviorItemClicked(QListWidgetItem*)));

    ui.playBehavior->setEnabled(false);
    ui.clearBehavior->setEnabled(false);
    ui.addPause->setEnabled(false);
    ui.stopRecord->setEnabled(false);
    ui.saveBehavior->setEnabled(false);

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
    string pauseMsg(to_string(pauseDuration) + "s pause");
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
}

void RoboyTrajectoriesControl::playTrajectoriesButtonClicked() {
    vector<string> actions = getCurrentActions();
    roboy_communication_control::PerformActions srv;
    srv.request.actions = actions;
    executeActionsServiceClient.call(srv);

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
