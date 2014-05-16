#include "ros/ros.h"

#include <termios.h>
#include <signal.h>
#include <sstream>

#include "dynamixel_control/GetIDs.h"
#include "dynamixel_control/SetActuatorsPositions.h"
#include "dynamixel_control/GetActuatorsLoads.h"

#include "std_msgs/String.h"

struct termios oldt, newt;

void quit(int sig)
{
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    ros::shutdown();

    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hexa_keyboard");
    ros::NodeHandle nh;

    ros::Publisher output_msg_pub = nh.advertise<std_msgs::String>("output_msg", 1000);

    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    signal(SIGINT,quit);

    ros::ServiceClient ids_client = nh.serviceClient<dynamixel_control::GetIDs>("/dynamixel_control/getids");
    dynamixel_control::GetIDs ids_srv;
    std::vector<unsigned char> ids;

    if (ids_client.call(ids_srv))
    {
        ids = ids_srv.response.ids;
        for(int i = 0; i < ids.size(); i++)
            ROS_INFO("%d", ids[i]);
    }
    else
    {
        ROS_ERROR("Failed to call service dynamixel_control/getids");
        return 1;
    }

    ros::ServiceClient setpos_client = nh.serviceClient<dynamixel_control::SetActuatorsPositions>("/dynamixel_control/setpositions");
    dynamixel_control::SetActuatorsPositions setpos_srv;

    std::vector<unsigned char> controlled_ids;
    std::vector<int> target_positions;

    for(int i = 0; i < ids.size(); i++)
    {
        if(ids[i] < 10)
        {
            controlled_ids.push_back(ids[i]);
            target_positions.push_back(2048);
        }
        else if(ids[i] < 30)
        {
            controlled_ids.push_back(ids[i]);
            target_positions.push_back(2048);
        }
        else if(ids[i] < 40)
        {
            controlled_ids.push_back(ids[i]);
            target_positions.push_back(512);
        }
    }
    setpos_srv.request.ids = controlled_ids;

    char c;
    bool new_value = true;

    std_msgs::String msg;
    std::stringstream msg_ss;

    ros::ServiceClient getloads_client = nh.serviceClient<dynamixel_control::GetActuatorsLoads>("/dynamixel_control/getloads");
    dynamixel_control::GetActuatorsLoads getloads_srv;
    std::vector<unsigned char> loads_ids;
    loads_ids.push_back(11);
    loads_ids.push_back(12);
    loads_ids.push_back(13);
    loads_ids.push_back(14);
    loads_ids.push_back(15);
    loads_ids.push_back(16);
    getloads_srv.request.ids = loads_ids;

    while (ros::ok())
    {
        if(!getloads_client.call(getloads_srv))
        {
            ROS_ERROR("Failed to call service dynamixel_control/getids");
            return 1;
        }

        msg_ss.str("");
        for(int i = 0; i < 3; i++)
            msg_ss << (int)(100*getloads_srv.response.loads[i]/1023.0) << "% ";
        msg.data = msg_ss.str();
	msg.data.resize(20);
	msg_ss.str("");
	for(int i = 3; i < 6; i++)
	    msg_ss << (int)(100*getloads_srv.response.loads[i]/1023.0) << "% ";
	msg.data += msg_ss.str();
	output_msg_pub.publish(msg);

        if(new_value)
        {
            setpos_srv.request.positions = target_positions;
            if(!setpos_client.call(setpos_srv))
            {
                ROS_ERROR("Failed to call service dynamixel_control/getids");
                return 1;
            }
            new_value = false;
        }

        if(read(STDIN_FILENO, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(c)
        {
        case 65:
            ROS_INFO("up");
            for(int i = 0; i < controlled_ids.size(); i++)
            {
                if(controlled_ids[i] < 10);
		else if(controlled_ids[i] >= 10 & controlled_ids[i] < 14)
                    target_positions[i] -= 100;
                else if(controlled_ids[i] < 20)
                    target_positions[i] += 100;
                else if(controlled_ids[i] >= 20 & controlled_ids[i] < 24)
                    target_positions[i] += 100;
                else if(controlled_ids[i] < 30)
                    target_positions[i] -= 100;

                if(target_positions[i] < 500)
                    target_positions[i] = 500;
                else if(target_positions[i] > 3500)
                    target_positions[i] = 3500;
            }
            new_value = true;
            break;
        case 66:
            ROS_INFO("down");
            for(int i = 0; i < controlled_ids.size(); i++)
            {
                if(controlled_ids[i] < 10);
		else if(controlled_ids[i] >= 10 & controlled_ids[i] < 14)
                    target_positions[i] += 100;
                else if(controlled_ids[i] < 20)
                    target_positions[i] -= 100;
                else if(controlled_ids[i] >= 20 & controlled_ids[i] < 24)
                    target_positions[i] -= 100;
                else if(controlled_ids[i] < 30)
                    target_positions[i] += 100;

                if(target_positions[i] < 500)
                    target_positions[i] = 500;
                else if(target_positions[i] > 3500)
                    target_positions[i] = 3500;
            }
            new_value = true;
            break;
        case 67:
            ROS_INFO("right");
            break;
        case 68:
            ROS_INFO("left");
            break;
        }

        ros::spinOnce();

    }

    return 0;
}
