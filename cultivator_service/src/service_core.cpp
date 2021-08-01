/*
15thARC　サービスコアノード
参考URL：https://github.com/ROBOTIS-GIT/turtlebot3_deliver/blob/master/turtlebot3_deliver_service/src/service_core.cpp
*/
#include <string.h>
#include <sstream>

#include "ros/ros.h"
#include "cultivator_service/PadOrder.h"
#include "cultivator_service/AveilableItemList.h"
#include "cultivator_service/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"

#define ROBOT_NUMBER_3P 0
#define ROBOT_NUMBER_3G 1
#define ROBOT_NUMBER_3R 2

class ServiceCore
{
    public:
        ServiceCore()
        {
            fnInitParam();  //マップ上でのロボットの目標位置姿勢を読み込む

            // 操縦者に状況を送信
            pubServiceStatusPad3p = nh_.advertise<std_msgs::String>("/3p/service_status", 1);
            pubServiceStatusPad3g = nh_.advertise<std_msgs::String>("/3g/service_status", 1);
            pubServiceStatusPad3r = nh_.advertise<std_msgs::String>("/3r/service_status", 1);

            pub_is_item_available = nh_.advertise<cultivator_service::AveilableItemList>("/is_item_available", 1);

            // 位置姿勢を送信
            pubPoseStamped3p = nh_.advertise<geometry_msgs::PoseStamped>("/3p/move_base_simple/goal", 1);
            pubPoseStamped3g = nh_.advertise<geometry_msgs::PoseStamped>("/3g/move_base_simple/goal", 1);
            pubPoseStamped3r = nh_.advertise<geometry_msgs::PoseStamped>("/3r/move_base_simple/goal", 1);

            //
            sub_pad_order_3p = nh_.subscribe("/3p/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
            sub_pad_order_3g = nh_.subscribe("/3g/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
            sub_pad_order_3r = nh_.subscribe("/3r/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
            
            // ロボットの到達成否を受信
            sub_arrival_status_3p = nh_.Subscribe("/3p/move_base/result", 1, &ServiceCore::cbCheckArrivalStatus3P, this);
            sub_arrival_status_3g = nh_.Subscribe("/3g/move_base/result", 1, &ServiceCore::cbCheckArrivalStatus3G, this);
            sub_arrival_status_3r = nh_.Subscribe("/3r/move_base/result", 1, &ServiceCore::cbCheckArrivalStatus3R, this);
            
            ros::Rate loop_rate(5);

            while(ros::ok())
            {
                fnPubServiceStatus();

                fnPubPose();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        /* マップ上でのロボットの目標位置姿勢を読み込む */
        void fnInitParam()
        {
            nh_.getParam("table_pose_3p/position", target_pose_position);
            nh_.getParam("table_pose_3p/orientation", target_pose_orientation);

            poseStampedTable[0].header.frame_id = "map";
            poseStampedTable[0].header.stamp = ros::Time::now();

            poseStampedTable[0].pose.position.x = target_pose_position[0];
            poseStampedTable[0].pose.position.y = target_pose_position[1];
            poseStampedTable[0].pose.position.z = target_pose_position[2];

            poseStampedTable[0].pose.orientation.x = target_pose_orientation[0];
            poseStampedTable[0].pose.orientation.y = target_pose_orientation[1];
            poseStampedTable[0].pose.orientation.z = target_pose_orientation[2];
            poseStampedTable[0].pose.orientation.w = target_pose_orientation[3];

            nh_.getParam("table_pose_3g/position", target_pose_position);
            nh_.getParam("table_pose_3g/orientation", target_pose_orientation);

            poseStampedTable[1].header.frame_id = "map";
            poseStampedTable[1].header.stamp = ros::Time::now();

            poseStampedTable[1].pose.position.x = target_pose_position[0];
            poseStampedTable[1].pose.position.y = target_pose_position[1];
            poseStampedTable[1].pose.position.z = target_pose_position[2];

            poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
            poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
            poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
            poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];

            nh_.getParam("table_pose_3r/position", target_pose_position);
            nh_.getParam("table_pose_3r/orientation", target_pose_orientation);

            poseStampedTable[2].header.frame_id = "map";
            poseStampedTable[2].header.stamp = ros::Time::now();

            poseStampedTable[2].pose.position.x = target_pose_position[0];
            poseStampedTable[2].pose.position.y = target_pose_position[1];
            poseStampedTable[2].pose.position.z = target_pose_position[2];

            poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
            poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
            poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
            poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];

            nh_.getParam("counter_pose/position", target_pose_position);
            nh_.getParam("counter_pose/orientation", target_pose_orientation);

            poseStampedCounter[0].header.frame_id = "map";
            poseStampedCounter[0].header.stamp = ros::Time::now();

            poseStampedCounter[0].pose.position.x = target_pose_position[0];
            poseStampedCounter[0].pose.position.y = target_pose_position[1];
            poseStampedCounter[0].pose.position.z = target_pose_position[2];

            poseStampedCounter[0].pose.orientation.x = target_pose_orientation[0];
            poseStampedCounter[0].pose.orientation.y = target_pose_orientation[1];
            poseStampedCounter[0].pose.orientation.z = target_pose_orientation[2];
            poseStampedCounter[0].pose.orientation.w = target_pose_orientation[3];
        }

        /* 受信したロボットの稼働状態を確認 */
        void cbCheckArrivalStatus3P(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
        {
            if(rcvMoveBaseActionResult.status.status == 3)
            {
                is_robot_reached_target[ROBOT_NUMBER_3P] = true;
            }
            else
            {
                ROS_INFO("cbCheckArrivalStatus3P : %d", rcvMoveBaseActionResult.status.status);
            }
        }

        void cbCheckArrivalStatus3G(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
        {
            if(rcvMoveBaseActionResult.status.status == 3)
            {
                is_robot_reached_target[ROBOT_NUMBER_3G] = true;
            }
            else
            {
                ROS_INFO("cbCheckArrivalStatus3G : %d", rcvMoveBaseActionResult.status.status);
            }
        }

        void cbCheckArrivalStatus3R(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
        {
            if(rcvMoveBaseActionResult.status.status == 3)
            {
                is_robot_reached_target[ROBOT_NUMBER_3R] = true;
            }
            else
            {
                ROS_INFO("cbCheckArrivalStatus3R : %d", rcvMoveBaseActionResult.status.status);
            }
        }

        /* ロボットが現在行っているサービスを監視し、目標位置に到着した後に次の目標位置を設定 */
        void fnPubPose()
        {
            // ロボットがナビゲーションの目的地に到達できたか
            if(is_robot_reached_target[ROBOT_NUMBER_3P])
            {
                if(robot_service_sequence[ROBOT_NUMBER_3P] == 1)
                {
                    robot_service_sequence[ROBOT_NUMBER_3P] == 2;
                }
                else if(robot_service_sequence[ROBOT_NUMBER_3P] == 2)
                {}
            }

            if(is_robot_reached_target[ROBOT_NUMBER_3G])
            {
                if(robot_service_sequence[ROBOT_NUMBER_3G] == 1)
                {
                    robot_service_sequence[ROBOT_NUMBER_3G] == 2;
                }
                else if(robot_service_sequence[ROBOT_NUMBER_3G] == 2)
                {}
            }

            if(is_robot_reached_target[ROBOT_NUMBER_3R])
            {
                if(robot_service_sequence[ROBOT_NUMBER_3R] == 1)
                {
                    robot_service_sequence[ROBOT_NUMBER_3R] == 2;
                }
                else if(robot_service_sequence[ROBOT_NUMBER_3R] == 2)
                {}
            }
        }

        void cbReceivePadOrder(const std_msgs::String padOrder)
        {
            std::string str = padOrder.data;
            std::string delimiter = ",";

            size_t pos = 0;

            int num = 0;
            int input_numbers[2] = {-2, -2};

            while((pos = str.find(delimiter)) != std::string::npos)
            {
                input_numbers[num] = atoi(str.subsr(0, pos).c_str());
                str.erase(0, pos + delimiter.length());

                num++;
            }

            input_numbers[num] = atoi(str.substr(0, str.size()).c_str());

            int pad_number = input_numbers[0];      //タブレット番号
            int item_number = input_numbers[1];     //注文品の番号

            if (is_item_available[item_number] != 1)
            {
                ROS_INFO("Chosen item is currently unavailable");
                return;
            }

            if (robot_service_sequence[pad_number] != 0)
            {
                ROS_INFO("Your TurtleBot is currently on servicing");
                return;
            }

            if (item_num_chosen_by_pad[pad_number] != -1)
            {
                ROS_INFO("Your TurtleBot is currently on servicing");
                return;
            }

            item_num_chosen_by_pad[pad_number] = item_number;
            robot_service_sequence[pad_number] = 1; // just left from the table
            is_item_available[item_number] = 0;
        }

        void fnPubServiceStatus()
        {
            std::string str;
            std_msgs::String serviceStatus;
            std::ostringstream oss;



            str = oss.str();
            serviceStatus.data = str;
            pubServiceStatusPad3p.publish(serviceStatus);
            pubServiceStatusPad3g.publish(serviceStatus);
            pubServiceStatusPad3r.publish(serviceStatus);
        }
    
    private:
        ros::NodeHandle nh_;

        // Publisher
        ros::Publisher pubServiceStatusPad3p;   // 操縦者に状況を送信
        ros::Publisher pubServiceStatusPad3g;
        ros::Publisher pubServiceStatusPad3r;
        ros::Publisher pub_is_item_available;

        ros::Publisher pubPoseStamped3p;        // 位置姿勢を送信
        ros::Publisher pubPoseStamped3g;
        ros::Publisher pubPoseStamped3r;

        // Subscriber
        ros::Subscriber sub_pad_order_3p;       // 注文
        ros::Subscriber sub_pad_order_3g;
        ros::Subscriber sub_pad_order_3r;

        ros::Subscriber sub_arrival_status_3p;  // ロボットの到達成否を受信
        ros::Subscriber sub_arrival_status_3g;
        ros::Subscriber sub_arrival_status_3r;

        // msgs
        geometry_msgs::PoseStamped poseStampedTable[3];
        geometry_msgs::PoseStamped poseStampedCounter[3];

        std::vector<double> target_pose_position;
        std::vector<double> target_pose_orientation;

        boost::array<int, 3> item_num_chosen_by_pad = {{-1, -1, -1}};
        boost::array<int, 3> is_item_available = {{1, 1, 1}};
        boost::array<int, 3> robot_service_sequence = {{0, 0, 0}};

        bool is_robot_reached_target[3] = {true, true, true}
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "service_core");

    //Create an object of class ServiceCore that will take care of everything
    ServiceCore serviceCore;

    ros::spin();

    return 0;
}