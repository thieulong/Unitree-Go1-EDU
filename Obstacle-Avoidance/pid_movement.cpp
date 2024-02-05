#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
// #include <vector>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"


// assume person is standing on the left of the robot
// diff_drive: following in differential fashion while obstacle avoiding in omni
// omni_drive: following and obstacle avoiding in omni

const float THRES_X = 0.2;
const float THRES_Y = 0.5;
const float THRES_Y_NEG = -THRES_Y;
const float TOLER_X = 0.2;
const float TOLER_Y = 0.4;
const float X_UP = THRES_X + TOLER_X;
const float X_LO = THRES_X - TOLER_X;
const float Y_R = THRES_Y - TOLER_Y;
const float Y_L = THRES_Y + TOLER_Y;
const float Y_R_NEG = THRES_Y_NEG - TOLER_Y;
const float Y_L_NEG = THRES_Y_NEG + TOLER_Y;

const float X_FAC = 0.5;
const float YAW_FAC = 1;
const float DBL_YAW_FAC = 1.5*YAW_FAC;
const float AVOID = 0.5;
const float degree = M_PI/90; 

const float PID_dt = 0.1

const float PID_V_kp = 0.2;
const float PID_V_ki = 0.05;
const float PID_V_kd = 0.085;

// const float tentacle[5][2] = {
//     // x, y
//     {0.1f,0.0f}, 
//     {0.1f,-0.1f}, 
//     {0.0f,0.1f},
//     {0.0f,-0.1f}, 
//     {0.1f,0.1f},    
// };
const float tentacle[8][2] = {
    // x, y
    {0.0f,-0.1f},
    {0.1f,-0.1f}, 
    {0.1f,0.0f}, 
    {0.1f,0.1f},
    {0.0f,0.1f},
    {-0.1f,0.1f},
    {-0.1f,0.0f},
    {-0.1f,-0.1f}
};
const int obst_rad = 0.5;
const float Hz = 10;
const float dt = 5*(1/Hz); 
const int rows = sizeof(tentacle)/sizeof(tentacle[0]);

using namespace UNITREE_LEGGED_SDK;

class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

    double calculate(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double output = kp * error + ki * integral + kd * derivative;
        prev_error = error;
        return output;
    }

private:
    double kp;
    double ki;
    double kd;
    double prev_error;
    double integral;
};

int main() {
    // Example usage:
    PIDController pid(0.1, 0.01, 0.05);
    double error = 10.0;
    double dt = 0.1;

    for (int i = 0; i < 10; ++i) {
        double output = pid.calculate(error, dt);
        std::cout << "Output: " << output << std::endl;

        // Simulate changing error in each iteration
        error -= 1.0;
    }

    return 0;
}


class Person_follow_diff
{
    public:
        
        Person_follow_diff():
            curx{0.0f},
            cury{0.0f},
            newx{0.0f},
            newy{0.0f},  
            diffx{0.0f},
            diffy{0.0f},  
            // obst_size{0},
            // obst_rad{0},
            pos{1},             
            // obst_x{0.0f},
            // obst_y{0.0f},
            // obst_x_other{0.0f},
            // obst_y_other{0.0f},
            // a{0},
            // b{0},
            // id{-1},  
            // obst_size{0}, 
            obst_avoided{true},
            angle{0},  
            drive{1},
            nh_{},
            pub_(nh_.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000)),
            pubobstacles_(nh_.advertise<std_msgs::Float64MultiArray>("obstacles", 1000)),
            // sub_(nh_.subscribe("/detected_objects", 1000, &Person_follow_diff::followCallback, this))
            // subperson_(nh_.subscribe("/person_coordinates", 1000, &Person_follow_diff::followCallback, this))
            subperson_(nh_.subscribe("/person_tracking", 1000, &Person_follow_diff::followCallback, this))
            // subobstacles_(nh_.subscribe("/obstacle_coordinates", 1000, &Person_follow_diff::obstaclesCallback, this))
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
            // obst_avoid = false;              
            // avoid_diff = false;
            
            
        };

        void obstaclesCallback(const std_msgs::Float64MultiArray &msg)
        {
            obst_array = msg;
            pubobstacles_.publish(obst_array); 
        }

        void diff_drive(const float &curx, const float &cury, const float &newx, const float &newy, unitree_legged_msgs::HighCmd high_cmd_ros)
        {
            diffx = 2*newx - curx;
            diffy = 2*newy - cury;
            PIDController pid(PID_V_kd, PID_V_ki, PID_V_kp);
            double error = 
            double velocity = pid.calculate(error, dt);
            high_cmd_ros.velocity[0] = X_FAC*sqrt(pow(diffx,2) + pow(diffy,2));     
            if (diffx > 0) high_cmd_ros.yawSpeed = YAW_FAC*atan(diffy/diffx);
            else if (diffx < 0) high_cmd_ros.yawSpeed = -DBL_YAW_FAC*atan(diffy/diffx);
            else high_cmd_ros.yawSpeed = abs(diffy)*YAW_FAC*(M_PI/2);  
        }

        void omni_drive(const float &curx, const float &cury, const float &newx, const float &newy, unitree_legged_msgs::HighCmd high_cmd_ros)
        {
            high_cmd_ros.velocity[0] = 1.5*newx - curx;
            high_cmd_ros.velocity[1] = 2*newy - cury; 
        }

        void spin_drive(const float &curx, const float &cury, const float &newx, const float &newy, unitree_legged_msgs::HighCmd high_cmd_ros)
        {
            high_cmd_ros.velocity[0] = 0.0f;
            high_cmd_ros.velocity[1] = 0.0f; 
            if (newx > 0) //*
            {
                do {
                    angle = atan(diffy/diffx);
                    high_cmd_ros.yawSpeed = YAW_FAC*angle;
                    pub_.publish(high_cmd_ros);
                } while (!(angle < degree && angle > -degree))
            }
            else if (newx < 0) 
            {
                do {
                    angle = atan(diffy/diffx);
                    high_cmd_ros.yawSpeed = -DBL_YAW_FAC*angle;
                    pub_.publish(high_cmd_ros);
                } while (!(angle < degree && angle > -degree))
            }
            else 
            {
                do {
                    angle = atan(diffy/diffx);
                    high_cmd_ros.yawSpeed = abs(diffy)*YAW_FAC*(M_PI/2);
                    pub_.publish(high_cmd_ros);
                } while (!(angle < degree && angle > -degree))                 
            }
            // diffx = 2*newx - curx;
            // diffy = 2*newy - cury; 
            high_cmd_ros.velocity[0] = X_FAC*sqrt(pow(newx,2) + pow(newy,2));     
                  
        }

        void followCallback(const std_msgs::Float64MultiArray &msg) 
        {            
            // a = 0; b = 0;
            // curx = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride*a + msg.layout.dim[2].stride*b];
            // a = 0; b = 1;
            // cury = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride*a + msg.layout.dim[2].stride*b];
            // a = 1; b = 0;
            // newx = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride*a + msg.layout.dim[2].stride*b];
            // a = 1; b = 1;
            // newy = msg.data[msg.layout.data_offset + msg.layout.dim[1].stride*a + msg.layout.dim[2].stride*b];  

            curx = msg.data[0];
            cury = msg.data[1];
            newx = msg.data[2];
            newy = msg.data[3];
            // id = msg.data[4]; 

            if (drive == 1) 
            {
                diff_drive(curx, cury, newx, newy, high_cmd_ros);
            }     
            else if (drive == 2)
            {
                omni_drive(curx, cury, newx, newy, high_cmd_ros);
            }
            else if (drive == 3)
            {
                spin_drive(curx, cury, newx, newy, high_cmd_ros);
            }
            else 
            {
                printf("Please specify a drive!");
            }          
            
          
            if (pos == 1) 
            {       
                if (curx < (X_UP+0.6) && curx > (X_LO+0.6) && (cury < Y_L || cury < Y_L_NEG) && (cury > Y_R || cury > Y_R_NEG)) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.yawSpeed = 0.0f;
                }
            }
            else if (pos == 2) 
            {  
                if (curx < X_UP && curx > X_LO && (cury < Y_L || cury < Y_L_NEG) && (cury > Y_R || cury > Y_R_NEG)) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.yawSpeed = 0.0f;
                }
            }
        
            
            // obstacle avoidance  
            // obst_size = obst_array.layout.dim[0].size;

            if (!obst_avoid)
            {
                // angle = atan(high_cmd_ros.velocity[1]/high_cmd_ros.velocity[0]);
                // high_cmd_ros.velocity[0] += 0.001;
                
                if (!obst_array.data[0] &  high_cmd_ros.velocity[1] < 0 &  high_cmd_ros.velocity[0] == 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[1] &  high_cmd_ros.velocity[1] < 0 &  high_cmd_ros.velocity[0] > 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[2] &  high_cmd_ros.velocity[1] == 0 &  high_cmd_ros.velocity[0] > 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[3] & high_cmd_ros.velocity[1] > 0 &  high_cmd_ros.velocity[0] > 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[4] & high_cmd_ros.velocity[1] > 0 &  high_cmd_ros.velocity[0] == 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[5] & high_cmd_ros.velocity[1] > 0 &  high_cmd_ros.velocity[0] < 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[6] & high_cmd_ros.velocity[1] == 0 &  high_cmd_ros.velocity[0] < 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                else if (!obst_array.data[7] & high_cmd_ros.velocity[1] < 0 &  high_cmd_ros.velocity[0] < 0) 
                {
                    high_cmd_ros.velocity[0] = 0.0f;
                    high_cmd_ros.velocity[1] = 0.0f;
                }
                // if (obst_size > 0)
                // {
                //     high_cmd_ros.velocity[0] = 0.0f;
                //     high_cmd_ros.velocity[1] = 0.0f;
                //     high_cmd_ros.yawSpeed = 0.0f;
                // }               
            }
            else
            {
                // switch (id) 
                // {
                //     case 0:
                //         high_cmd_ros.velocity[0] = -1.0f*AVOID;
                //         break;
                //     case 1:
                //         high_cmd_ros.velocity[0] = 1.0f*AVOID;
                //         break;
                //     case 2:
                //         high_cmd_ros.velocity[1] = -1.0f*AVOID;
                //         break;
                //     case 3:
                //         high_cmd_ros.velocity[1] = 1.0f*AVOID;
                //         break;
                //     default:
                //         high_cmd_ros.velocity[0] = high_cmd_ros.velocity[0];
                //         high_cmd_ros.velocity[1] = 0.0f;
                //         high_cmd_ros.yawSpeed = high_cmd_ros.yawSpeed;
                // }                          
                // if (obst_size > 0)
                if (std::find(std::begin(obst_array),std::end(obst_array),1) != std::end(obst_array))
                {
                    avoid(high_cmd_ros, obst_array, newx, newy);
                    obst_avoided = false;
                }   
                else 
                {
                    if (!obst_avoided)
                    {
                        obst_avoided = true;
                                              
                        high_cmd_ros.velocity[0] = X_FAC*sqrt(pow(newx,2) + pow(newy,2));     
                        if (newx > 0) high_cmd_ros.yawSpeed = YAW_FAC*atan(newy/newx);
                        else if (newx < 0) high_cmd_ros.yawSpeed = -DBL_YAW_FAC*atan(newy/newx);
                        else high_cmd_ros.yawSpeed = abs(newy)*YAW_FAC*(M_PI/2);
                    }
                    
                }                   
            }             


            if  
            pub_.publish(high_cmd_ros);
        };

        void avoid(unitree_legged_msgs::HighCmd &highcmd, const std_msgs::Float64MultiArray &coord, const float &newx, const float &newy) 
        {
            
            float future_x = 0.0f; 
            float future_y = 0.0f;
            float cost = 0.0;
            float best_cost = 1000.0f;
            int move = rows;

            // if (newy > 0)
            // {
            //     for (int i = 3; i <= rows-3; i++)
            //     {
            //         future_x = tentacle[i][0]*dt;
            //         future_y = tentacle[i][1]*dt;
            //         for (int j = 0; j < obst_size; j++)
            //         {                     
            //             obst_x = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*0];
            //             obst_y = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*1];
            //             obst_x = coord.data[0];
            //             obst_y = coord.data[1];
            //             obst_x_other = coord.data[2];
            //             obst_y_other = coord.data[3];
            //             if (pow(future_x - obst_x,2) + pow(future_y - obst_y,2) > obst_rad)
                        
            //             {
            //                 cost =  sqrt(pow(future_x - newx,2) + pow(future_y - newy,2));
            //                 if (cost < best_cost) 
            //                 {
            //                     best_cost = cost;
            //                     move = i;
            //                 }
            //             }                  
            //         }                
            //     }
            // }
            // else if (newy < 0)
            // {
            //     for (int i = 7; i % 8 <= 1; i++)
            //     {
            //         future_x = tentacle[i][0]*dt;
            //         future_y = tentacle[i][1]*dt;
            //         for (int j = 0; j < obst_size; j++)
            //         {                       
            //             obst_x = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*0];
            //             obst_y = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*1];
            //             if (pow(future_x - obst_x,2) + pow(future_y - obst_y,2) > obst_rad)
            //             {
            //                 cost =  sqrt(pow(future_x - newx,2) + pow(future_y - newy,2));
            //                 if (cost < best_cost) 
            //                 {
            //                     best_cost = cost;
            //                     move = i;
            //                 }
            //             }                  
            //         }                
            //     }
            // }
            // else 
            // {
            //     if (newx > 0) 
            //     {
            //         for (int i = 1; i <= 3; i++)
            //         {
            //             future_x = tentacle[i][0]*dt;
            //             future_y = tentacle[i][1]*dt;
            //             for (int j = 0; j < obst_size; j++)
            //             {                       
            //                 obst_x = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*0];
            //                 obst_y = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*1];
            //                 if (pow(future_x - obst_x,2) + pow(future_y - obst_y,2) > obst_rad)
            //                 {
            //                     cost =  sqrt(pow(future_x - newx,2) + pow(future_y - newy,2));
            //                     if (cost < best_cost) 
            //                     {
            //                         best_cost = cost;
            //                         move = i;
            //                     }
            //                 }                  
            //             }                
            //         }
            //     }
            //     else if (newx < 0)
            //     {
            //         for (int i = 5; i <= 7; i++)
            //         {
            //             future_x = tentacle[i][0]*dt;
            //             future_y = tentacle[i][1]*dt;
            //             for (int j = 0; j < obst_size; j++)
            //             {                       
            //                 obst_x = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*0];
            //                 obst_y = coord.data[coord.layout.data_offset + coord.layout.dim[1].stride*j + coord.layout.dim[2].stride*1];
            //                 if (pow(future_x - obst_x,2) + pow(future_y - obst_y,2) > obst_rad)
            //                 {
            //                     cost =  sqrt(pow(future_x - newx,2) + pow(future_y - newy,2));
            //                     if (cost < best_cost) 
            //                     {
            //                         best_cost = cost;
            //                         move = i;
            //                     }
            //                 }                  
            //             }                
            //         }
            //     }
            //     else 
            //     {
            //         printf("Person cannot be on top of robot!");
            //     }
            // }
            
            for (int i == 0; i < obst_size; i++)
            {
                if (obst_array[i])
                {
                    future_x = tentacle[i][0]*dt;
                    future_y = tentacle[i][1]*dt;
                }   
                cost =  sqrt(pow(future_x - newx,2) + pow(future_y - newy,2));
                if (cost < best_cost) 
                {
                    best_cost = cost;
                    move = i;
                }
            }

            if (avoid_diff)
            {
                switch (move) 
                {
                    case 0:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = -1.0f*AVOID*2;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;
                    case 1:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = -0.5f*AVOID;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;
                    case 2:
                        highcmd.velocity[0] = 1.0f*AVOID;
                        break;
                    case 3:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = 0.5f;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;
                    case 4:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = 1.0f*AVOID*2;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;
                    case 5:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = 2.0f*AVOID*2;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;
                    case 6:
                        highcmd.velocity[0] = -1.0f*AVOID;
                        break;
                    case 7:
                        highcmd.velocity[0] = 0.5f*AVOID;
                        highcmd.yawSpeed = -2.0f*AVOID*2;
                        pub_.publish(highcmd);
                        highcmd.velocity[0] = 0.5f*AVOID;
                        break;            
                    default:
                        highcmd.velocity[0] = 0.0f;
                        highcmd.velocity[1] = 0.0f;
                        highcmd.yawSpeed = newy*0.5f;
                        printf("Trying to find a way!");
                }
            }
            else 
            {
                if (move != rows)
                {
                    highcmd.velocity[0] = tentacle[move][0];
                    highcmd.velocity[1] = tentacle[move][1]; 
                }
                else 
                {
                    highcmd.velocity[0] = 0.0f;
                    highcmd.velocity[1] = 0.0f; 
                    highcmd.yawSpeed = newy*0.3f;                
                    
                    printf("Trying to find a way!");
                }
            }
            
              
        }

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        // ros::Publisher pubobstacles_;
        ros::Subscriber subperson_;
        // ros::Subscriber subobstacles_;

        

        float curx;
        float cury;
        float newx;
        float newy;
        float diffx;
        float diffy;
        unitree_legged_msgs::HighCmd high_cmd_ros;
        // float obst_rad;
        int pos; // person in 1: front, 2: side, 
        // int32_t id; 

        int obst_size;
        bool obst_avoid; 
        // float obst_x;
        // float obst_y;
        // float obst_x_other;
        // float obst_y_other;
        std_msgs::Float64MultiArray obst_array;// *
        bool avoid_diff; // diff, omni
        bool obst_avoided;

        // int a;
        // int b;
        float angle;
        int drive; // 1: diff, 2: omni, 3: spin


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Person_follow_diff_cpp");

    Person_follow_diff Person_follow_diff;
   
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