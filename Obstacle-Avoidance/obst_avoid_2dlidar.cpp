#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <cmath>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"


//make sure to run $ roslaunch unitree_legged_real real.launch ctrl_level:=highlevel before this file

// assume person is standing on the left and in front of the robot
const float THRES_X = 0.2;
const float THRES_Y = 0.5;
const float TOLER_X = 0.2;
const float TOLER_Y = 0.4;
const float X_UP = THRES_X + TOLER_X;
const float X_LO = THRES_X - TOLER_X;
const float Y_R = THRES_Y - TOLER_Y;
const float Y_L = THRES_Y + TOLER_Y;

const float X_FAC = 0.5;
const float YAW_FAC = 1;
const float DBL_YAW_FAC = 1.5*YAW_FAC;

using namespace UNITREE_LEGGED_SDK;

class Obst_avoid_2d
{
    public:
        Obst_avoid_2d():
            curx{0},
            cury{0},
            newx{0},
            newy{0},  
            diffx{0},
            diffy{0},  
            id{-1}, 
            nh_{},
            pub_(nh_.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000)),
            sub_(nh_.subscribe("/obstacles", 1000, &Obst_avoid_2d::avoidCallback, this))
        { 
            high_cmd_ros.head[0] = 0xFE;
            high_cmd_ros.head[1] = 0xEF;
            high_cmd_ros.levelFlag = HIGHLEVEL;
            high_cmd_ros.mode = 2; 
            high_cmd_ros.gaitType = 1;
            high_cmd_ros.speedLevel = 0;
            high_cmd_ros.footRaiseHeight = 0;
            high_cmd_ros.bodyHeight = 0;
            high_cmd_ros.euler[0] = 0;
            high_cmd_ros.euler[1] = 0;
            high_cmd_ros.euler[2] = 0;
            high_cmd_ros.reserve = 0;
        };

        void avoidCallback(const std_msgs::Int32 &msg) 
        {
            id = msg.data;    

            
            if (id == 0) 
            {
                high_cmd_ros.velocity[0] = -1.0f;
                high_cmd_ros.velocity[1] = 0.0f;
            }
            else if (id == 1) 
            {
                high_cmd_ros.velocity[0] = 1.0f;
                high_cmd_ros.velocity[1] = 0.0f;
            }
            else if (id == 2) 
            {
                high_cmd_ros.velocity[0] = 0.0f;
                high_cmd_ros.velocity[1] = -1.0f;
            }
            else if (id == 3) 
            {
                high_cmd_ros.velocity[0] = 0.0f;
                high_cmd_ros.velocity[1] = 1.0f;
            } 
            else 
            {
                high_cmd_ros.velocity[0] = 0.0f;
                high_cmd_ros.velocity[1] = 0.0f;
            }
                
            pub_.publish(high_cmd_ros);
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;

        unitree_legged_msgs::HighCmd high_cmd_ros;

        float curx;
        float cury;
        float newx;
        float newy;
        float diffx;
        float diffy;
        int32_t id; 


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obst_avoid_2d_cpp");

    Obst_avoid_2d obst_avoid_2d;
   
    ros::spin();
    return 0;
}


 // if (cury >= Y_L)
// {
//     if (curx < X_LO) 
//     {
//         high_cmd_ros.velocity[0] = curx/X_FAC;
//         high_cmd_ros.yawSpeed = 2.0/ YAW_FAC;
//         pub_.publish(high_cmd_ros);
//         while (cury != Y_L) {;}
        
//     }
//     else if (curx > X_UP) 
//     {
//         high_cmd_ros.velocity[0] = (curx - X_UP)/ X_FAC;
//         high_cmd_ros.yawSpeed = (cury - Y_L)/ YAW_FAC;                
//     }
//     else 
//     {
//         high_cmd_ros.velocity[0] = 0.5f;
//         high_cmd_ros.yawSpeed = 0.8f;
//     }
    
// }
// else if (cury <= Y_R)
// {
//     if (curx < X_LO) 
//     {
//         high_cmd_ros.velocity[0] = 0.0f;
//         high_cmd_ros.yawSpeed = 0.0f;
//     }
//     else if (curx > X_UP) 
//     {
//         high_cmd_ros.velocity[0] = (curx - X_UP)/ X_FAC;
//         high_cmd_ros.yawSpeed = (cury - Y_R)/ YAW_FAC;                
//     }
//     else 
//     {
//         high_cmd_ros.yawSpeed = -1.0f;
//     }
// }
// else 
// {
//     high_cmd_ros.yawSpeed = 0.0f;
//     if (curx < X_LO) high_cmd_ros.velocity[0] = (curx - X_LO)/ X_FAC;
//     else if (curx > X_UP) high_cmd_ros.velocity[0] = (curx - X_UP)/ X_FAC;
//     else high_cmd_ros.velocity[0] = 0;
// }