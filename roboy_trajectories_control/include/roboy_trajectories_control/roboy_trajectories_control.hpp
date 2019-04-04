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
#include <roboy_control_msgs/StartRecordTrajectory.h>
#include <roboy_control_msgs/PerformMovementAction.h>
#include <roboy_control_msgs/PerformMovementsAction.h>
#include <roboy_control_msgs/PerformMovementsActionResult.h>
#include <roboy_control_msgs/ListItems.h>
#include <roboy_control_msgs/Behavior.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/SetInt16.h>
#include <actionlib/client/simple_action_client.h>

#endif

using namespace std;
typedef actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementsAction> MovementsAC;
typedef actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementAction> MovementAC;

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
            void saveBehavior(string name, vector<string> actions);


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
            void allToDisplacementButtonClicked();
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
            void setDisplacementOfAllSelectedMotors(int displacement);
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
                    performMovementServiceClient,
                    executeActionsServiceClient, listExistingTrajectoriesServiceClient;
            map<string,ros::ServiceClient> setDisplacementForAllServiceClient;

//            ros::ServiceClient listExistingBehaviorsServiceClient, expandBehaviorServiceClient;

//            actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementsAction> lsh_movements_ac, rsh_movements_ac,
//                    legs_movements_ac, lsp_movements_ac, rsp_movements_ac, head_movements_ac;
//            actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementAction> lsh_movement_ac, rsh_movement_ac,
//                    legs_movement_ac, lsp_movement_ac, rsp_movement_ac, head_movement_ac;

            map<string,actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementAction>*> performMovement_ac;
            map<string,actionlib::SimpleActionClient<roboy_control_msgs::PerformMovementsAction>*> performMovements_ac;
            void checkMotorStatus();
            void motorStatusCallback(const roboy_middleware_msgs::MotorStatus::ConstPtr &msg);
            void performMovementsResultCallback(const roboy_control_msgs::PerformMovementsActionResult::ConstPtr &msg);
            void initializeRosCommunication();

        private:
            bool stopButton;
            int pauseDuration; // in seconds
            int preDisplacement = 50;
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
            vector<QCheckBox*> fpgaCheckBox;
            //TODO refactor fpgaBodypart
            vector<string> fpgaBodyPart;



};
