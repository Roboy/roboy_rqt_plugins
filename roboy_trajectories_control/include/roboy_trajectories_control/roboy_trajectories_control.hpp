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
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
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

#define HEAD 0
#define SPINE_LEFT 1
#define SPINE_RIGHT 2
#define LEGS 3
// the two shoulders have to have these ids, because the right shoulder has mirrored motor units, which results in
// negative displacement on compression of the spring and needs to be dealt with in fpga PID controllers
#define SHOULDER_LEFT 4
#define SHOULDER_RIGHT 5

#endif

using namespace std;
typedef actionlib::SimpleActionClient<roboy_communication_control::PerformMovementsAction> MovementsAC;
typedef actionlib::SimpleActionClient<roboy_communication_control::PerformMovementAction> MovementAC;

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

            QGraphicsScene* scene;
            vector<string> listExistingBehaviors();
            vector<string> expandBehavior(string name);
            void saveBehavior(string name, vector<string> action);


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
            void addSyncButtonClicked();
            void relaxAllMusclesButtonClicked();
            void startInitializationButtonClicked();
            void startRecordTrajectoryButtonClicked();
            void stopRecordTrajectoryButtonClicked();
            void stopBehaviorButtonClicked();
            void saveBehaviorButtonClicked();
            void loadBehaviorButtonClicked();
            void setPauseDuration(int duration);
            void setTimeUnits(int idx);
            void setPredisplacementButtonClicked();
            void setPredisplacement(int value);
        private:
            Ui::RoboyTrajectoriesControl ui;
            QWidget *widget_;
            ros::NodeHandlePtr nh;
            ros::Publisher motorCommandPublisher, startRecordTrajectoryPublisher,
                    stopRecordTrajectoryPublisher, saveBehaviorPublisher, enablePlaybackPublisher,
                    preDisplacementPublisher;
            ros::Subscriber motorStatusSubscriber, jointStatusSubscriber, motorCommandSubscriber;
            map<string,ros::Subscriber> performMovementsResultSubscriber;
            map<string,ros::ServiceClient> motorControlServiceClient, emergencyStopServiceClient,
                    performMovementServiceClient, setDisplacementForAllServiceClient,
                    executeActionsServiceClient, listExistingTrajectoriesServiceClient;
//            ros::ServiceClient listExistingBehaviorsServiceClient, expandBehaviorServiceClient;

//            actionlib::SimpleActionClient<roboy_communication_control::PerformMovementsAction> lsh_movements_ac, rsh_movements_ac,
//                    legs_movements_ac, lsp_movements_ac, rsp_movements_ac, head_movements_ac;
//            actionlib::SimpleActionClient<roboy_communication_control::PerformMovementAction> lsh_movement_ac, rsh_movement_ac,
//                    legs_movement_ac, lsp_movement_ac, rsp_movement_ac, head_movement_ac;

            map<string,actionlib::SimpleActionClient<roboy_communication_control::PerformMovementAction>*> performMovement_ac;
            map<string,actionlib::SimpleActionClient<roboy_communication_control::PerformMovementsAction>*> performMovements_ac;
            void checkMotorStatus();
            void motorStatusCallback(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
            void performMovementsResultCallback(const roboy_communication_control::PerformMovementsActionResult::ConstPtr &msg);
            void initializeRosCommunication();

        private:
            const vector<string> bodyParts = {"head",  "spine_left",  "spine_right", "legs", "shoulder_left", "shoulder_right"};
            bool stopButton;
            int pauseDuration; // in seconds
            int preDisplacement;
            int timeFactor=1;
            vector<double> setpoint;
            vector<int> control_mode;
            vector<int> total_number_of_motors;
//            number_of_fpgas = 1;
            vector<QRadioButton*> pos, vel, dis, force;
            vector<QSlider*> setpoint_slider_widget;
            vector<QLineEdit*> setpoint_widget;
            QLineEdit* scale;
            const string trajectories_path = "/home/root/trajectories/";
            const string behaviors_path = "/home/roboy/behaviors/";
            vector<string> getCurrentActions();
            QBrush greenBrush;//(Qt::green);
            QBrush redBrush;//(Qt::red);
            vector<vector<bool>> motorStatus; // motor status
            vector<QCheckBox*> activeBodyParts;



};
