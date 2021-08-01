/*
引用URL：https://github.com/ROBOTIS-GIT/turtlebot3_deliver/blob/master/turtlebot3_deliver_service/src/pad_virtual.cpp
*/

#include "ros/ros.h"
#include "cultivator_service/PadOrder.h"
#include "cultivator_service/ServiceStatus.h"

class PadVirtual
{
    public:
        PadVirtual()
        {
            ros::param::get("~robot_num", robot_num);
            pub_pad_order = nh_.advertise<cultivator_service::PadOrder>("pad_order", 1);
            subServiceStatus = nh_.subscribe("service_status", 1, &PadVirtual::cbCheckServiceStatus, this);

            fnPubPadOrder();
        }

        void chCheckServiceStatus(const cultivator_service::ServiceStatus rcvServiceStatus)
        {
            item_num_chosen_by_pad = rcvServiceStatus.item_num_chosen_by_pad;
            is_item_available = rcvServiceStatus.is_item_available;
            robot_service_sequence = rcvServiceStatus.robot_service_sequence;
        }

        void fnPubPadOrder()
        {
            ros::Rate loop_rate(1);

            int selected_item_num = -1;

            while(ros::ok())
            {
                cultivator_service::PadOrder padOrder;

                std::string inputString;
                //std::cout << "Which item would you choose? (0. Bread 1. Drink 2. Snack or q. Quit)" << '\n';
                std::getline(std::cin, inputString);

                if (inputString.compare("0") == 0)
                {}
                else if(inputString.compare("q") == 0)
                {
                    return;
                }
                else
                {
                    ROS_INFO("Sorry, selected item is now unavailable. Please choose another item");
                    goto spin;
                }

                if (!is_item_available[selected_item_num])
                {
                    ROS_INFO();
                    goto spin;
                }

                if (robot_service_sequence != 0)
                {
                    ROS_INFO();
                    goto spin;
                }

                if (item_num_chosen_by_pad != -1)
                {
                    ROS_INFO();
                    goto spin;
                }

                padOrder.pad_number = robot_num;
                padOrder.item_number = selected_item_num;

                pub_pad_order.publish(padOrder);

                spin;

                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        ros::NodeHandle nh_;

        // Publisher
        ros::Publisher pub_pad_order;

        // Subscriber
        ros::Subscriber subServiceStatus;

        boost::array<int, 3> item_num_chosen_by_pad = {{-1, -1, -1}};
        boost::array<bool, 3> is_item_available = {{true, true, true}};
        boost::array<int, 3> robot_service_sequence = {{0, 0, 0}};

        int robot_num;
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "pad_virtual");

    //Create PadVirtual class
    PadVirtual padVirtual;

    ros::spin();

    return 0;
}