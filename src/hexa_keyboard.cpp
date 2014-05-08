#include "ros/ros.h"

#include <termios.h>
#include <signal.h>

#include "dynamixel_control/GetIDs.h"
#include "dynamixel_control/SetActuatorsPositions.h"

#include "phidget21.h"

int CCONV AttachHandler(CPhidgetHandle TXT, void *userptr)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (TXT, &name);
    CPhidget_getSerialNumber(TXT, &serialNo);
    ROS_INFO("%s %10d attached!\n", name, serialNo);

    return 0;
}

int CCONV DetachHandler(CPhidgetHandle TXT, void *userptr)
{
    int serialNo;
    const char *name;

    CPhidget_getDeviceName (TXT, &name);
    CPhidget_getSerialNumber(TXT, &serialNo);
    ROS_INFO("%s %10d detached!\n", name, serialNo);

    return 0;
}

int CCONV ErrorHandler(CPhidgetHandle TXT, void *userptr, int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s\n", ErrorCode, Description);
    return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_properties(CPhidgetTextLCDHandle phid)
{
    int serialNo, version, numRows, numColumns, backlight, cursor, contrast, cursor_blink, numScreens;
    const char* ptr;
    CPhidget_DeviceID id;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
    CPhidget_getDeviceID((CPhidgetHandle)phid, &id);

    CPhidgetTextLCD_getRowCount (phid, &numRows);
    CPhidgetTextLCD_getColumnCount (phid, &numColumns);
    CPhidgetTextLCD_getBacklight (phid, &backlight);
    CPhidgetTextLCD_getContrast (phid, &contrast);
    CPhidgetTextLCD_getCursorOn (phid, &cursor);
    CPhidgetTextLCD_getCursorBlink (phid, &cursor_blink);

    ROS_INFO("%s\n", ptr);
    ROS_INFO("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
    if(id == PHIDID_TEXTLCD_ADAPTER){
        CPhidgetTextLCD_getScreenCount (phid, &numScreens);
        ROS_INFO("# Screens: %d\n", numScreens);
        CPhidgetTextLCD_setScreen(phid, 0);
        CPhidgetTextLCD_setScreenSize(phid, PHIDGET_TEXTLCD_SCREEN_2x16);
        CPhidgetTextLCD_initialize(phid);
    }

    ROS_INFO("# Rows: %d\n# Columns: %d\n", numRows, numColumns);
    ROS_INFO("Current Contrast Level: %d\nBacklight Status: %d\n", contrast, backlight);
    ROS_INFO("Cursor Status: %d\nCursor Blink Status: %d\n", cursor, cursor_blink);

    return 0;
}

struct termios oldt, newt;

CPhidgetTextLCDHandle lcd_handle;

void quit(int sig)
{
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    ros::shutdown();

    CPhidget_close((CPhidgetHandle)lcd_handle);
    CPhidget_delete((CPhidgetHandle)lcd_handle);

    exit(0);
}

int main(int argc, char **argv)
{
    int result;
    CPhidgetTextLCD_create(&lcd_handle);
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)lcd_handle, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)lcd_handle, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)lcd_handle, ErrorHandler, NULL);
    CPhidget_open((CPhidgetHandle)lcd_handle,-1);
    const char* err;
    if((result = CPhidget_waitForAttachment((CPhidgetHandle)lcd_handle, 10000)))
    {
        CPhidget_getErrorDescription(result, &err);
        ROS_ERROR("Problem waiting for attachment: %s\n", &err);
        return 0;
    }
    display_properties(lcd_handle);
    CPhidgetTextLCD_setContrast (lcd_handle, 255);

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
            if(!CPhidgetTextLCD_setDisplayString(lcd_handle, 0, "UP     "))
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
            if(!CPhidgetTextLCD_setDisplayString(lcd_handle, 0, "DOWN   "))
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
