#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <roboy_trajectories_control/ui_roboy_trajectories_control.h>
#include <QWidget>
#include <QtWidgets/QtWidgets>
#include <QtQuick/QQuickView>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QScrollArea>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QLabel>
#include <QListWidget>
#include <QTextEdit>
#include <QMessageBox>
#include <QBrush>
#include <map>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <std_srvs/SetBool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <roboy_communication_control/StartRecordTrajectory.h>
//#include <roboy_communication_control/StopRecordTrajectoryAction.h>
#include <roboy_communication_control/StartRecordTrajectoryAction.h>
//#include <roboy_communication_control/StopRecordTrajectory.h>
#include <roboy_communication_control/PerformMovement.h>
#include <roboy_communication_control/PerformMovementAction.h>
#include <roboy_communication_control/PerformMovementsAction.h>
#include <roboy_communication_control/PerformMovementsActionResult.h>
#include <roboy_communication_control/PerformBehavior.h>
#include <roboy_communication_control/PerformActions.h>
#include <roboy_communication_control/ListItems.h>
#include <roboy_communication_control/Behavior.h>
#include <roboy_communication_middleware/MotorCommand.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/SetInt16.h>
#include <actionlib/client/simple_action_client.h>

#endif

using namespace std;

class RoboyTrajectoriesControl:
        public rqt_gui_cpp::Plugin, MotorConfig {
        Q_OBJECT
        public:
            RoboyTrajectoriesControl();

            virtual void initPlugin(qt_gui_cpp::PluginContext &context);

            virtual void shutdownPlugin();

            virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                      qt_gui_cpp::Settings &instance_settings) const;

            virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                         const qt_gui_cpp::Settings &instance_settings);

        public Q_SLOTS:
            void pullExistingTrajectories();
            void pullExistingBehaviors();
            void onExistingTrajectoriesItemClicked(QListWidgetItem* item);
            void onScheduledBehaviorItemClicked(QListWidgetItem* item);
            void refreshTrajectoriesButtonClicked();
            void playTrajectoriesButtonClicked();
            void clearAllTrajectoriesButtonClicked();
            void addPauseButtonClicked();
            void addRelaxButtonClicked();
            void relaxAllMusclesButtonClicked();
            void startInitializationButtonClicked();
            void startRecordTrajectoryButtonClicked();
            void stopRecordTrajectoryButtonClicked();
            void stopBehaviorButtonClicked();
            void saveBehaviorButtonClicked();
            void loadBehaviorButtonClicked();
            void setPauseDuration(int duration);
            void setTimeUnits(int idx);
        private:
            Ui::RoboyTrajectoriesControl ui;
            QWidget *widget_;
            vector<QGraphicsView*> motorStatusViews;
            ros::NodeHandlePtr nh;
            ros::Publisher motorCommandPublisher, startRecordTrajectoryPublisher,
                    stopRecordTrajectoryPublisher, saveBehaviorPublisher, enablePlaybackPublisher;
            ros::Subscriber motorStatusSubscriber, jointStatusSubscriber, motorCommandSubscriber,
                    performMovementsResultSubscriber;
            ros::ServiceClient motorControlServiceClient, emergencyStopServiceClient,
                    performMovementServiceClient, setDisplacementForAllServiceClient,
                    executeActionsServiceClient, listExistingTrajectoriesServiceClient,
                    listExistingBehaviorsServiceClient, expandBehaviorServiceClient;
            actionlib::SimpleActionClient<roboy_communication_control::PerformMovementAction> performMovement_ac;
            actionlib::SimpleActionClient<roboy_communication_control::PerformMovementsAction> performMovements_ac;
            void checkMotorStatus();
            void motorStatusCallback(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
            void performMovementsResultCallback(const roboy_communication_control::PerformMovementsActionResult::ConstPtr &msg);

        private:
            bool stopButton;
            int pauseDuration; // in seconds
            int timeFactor=1;
            vector<double> setpoint;
            vector<int> control_mode;
            int total_number_of_motors = 6, number_of_fpgas = 1;
            vector<QRadioButton*> pos, vel, dis, force;
            vector<QSlider*> setpoint_slider_widget;
            vector<QLineEdit*> setpoint_widget;
            QLineEdit* scale;
            const string trajectories_path = "/home/root/trajectories/";
            const string behaviors_path = "/home/root/behaviors/";
            vector<string> getCurrentActions();
            vector<string> expandBehavior(string name);
            QBrush greenBrush;//(Qt::green);
            QBrush redBrush;//(Qt::red);
            vector<bool> motorOnline; // motor status


};
