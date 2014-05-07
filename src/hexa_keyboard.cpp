#include "ros/ros.h"

#include <termios.h>
#include <signal.h>

#include "dynamixel_control/GetIDs.h"
#include "dynamixel_control/SetActuatorsPositions.h"

#include "phidget21.h"

struct termios oldt, newt;

CPhidgetTextLCDHandle lcd_handle;

void quit(int sig)
{
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ROS_INFO("LCD Create : %d",CPhidgetTextLCD_create(&lcd_handle));
    CPhidget_open((CPhidgetHandle)lcd_handle,-1);
    CPhidgetTextLCD_setScreenSize(lcd_handle,PHIDGET_TEXTLCD_SCREEN_2x20);
    int backlight = 0;
    CPhidgetTextLCD_getBacklight(lcd_handle,&backlight);    
    ROS_INFO("Backlight : %d",backlight);
    CPhidgetTextLCD_setBacklight(lcd_handle,255);
    CPhidgetTextLCD_initialize(lcd_handle);

    ros::init(argc, argv, "hexa_keyboard");
    ros::NodeHandle nh;

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
        if(ids[i] < 10);
       /* {
            controlled_ids.push_back(ids[i]);
            target_positions.push_back(0);
        }*/
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

    char c;
    bool new_value = true;
    while (ros::ok())
    {
        if(read(STDIN_FILENO, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        // debattement max = 1450 pour moteur 2x
        switch(c)
        {
        case 65:
            ROS_INFO("up");
            if(!CPhidgetTextLCD_setDisplayString(lcd_handle, 0, "UP"))
                ROS_ERROR("Failed to display text");
            for(int i = 0; i < controlled_ids.size(); i++)
            {
                if(controlled_ids[i] >= 10 & controlled_ids[i] < 20)
                {
                    target_positions[i]-=100;
                    if(target_positions[i] < 500)
                        target_positions[i] = 500;
                }
                else if(controlled_ids[i] >= 20 & controlled_ids[i] < 30)
                {
                    target_positions[i]+=100;
                    if(target_positions[i] > 3500)
                        target_positions[i] = 3500;
                }
            }
            new_value = true;
            break;
        case 66:
            ROS_INFO("down");
            if(!CPhidgetTextLCD_setDisplayString(lcd_handle, 0, "DOWN"))
                ROS_ERROR("Failed to display text");
            for(int i = 0; i < controlled_ids.size(); i++)
            {
                if(controlled_ids[i] >= 10 & controlled_ids[i] < 20)
                {
                    target_positions[i]+=100;
                    if(target_positions[i] > 3500)
                        target_positions[i] = 3500;
                }
                else if(controlled_ids[i] >= 20 & controlled_ids[i] < 30)
                {
                    target_positions[i]-=100;
                    if(target_positions[i] < 500)
                        target_positions[i] = 500;
                }
            }
            new_value = true;
            break;
        case 67:
            ROS_INFO("right");
            new_value = true;
            break;
        case 68:
            ROS_INFO("left");
            new_value = true;
            break;
        }

        if(new_value)
        {
            setpos_srv.request.ids = controlled_ids;
            setpos_srv.request.positions = target_positions;
            if(!setpos_client.call(setpos_srv))
            {
                ROS_ERROR("Failed to call service dynamixel_control/getids");
                return 1;
            }
            new_value = false;
        }

        ros::spinOnce();

    }

    return 0;
}
