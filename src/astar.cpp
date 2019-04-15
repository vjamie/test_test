///
///
///


// essential header for ROS-OpenCV operation
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

// for publishing and subscribing to images in ROS
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/CompressedImage.h>
//#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

//Define for ROS
#define ROS_waitkey 50          // Define waitkey if it is needed
#define Hz 20                   // Define loop period
#define D2R             0.01745329251994329576923690768489

///Define variables
// Definition of LL Generator
#define RANGE           50      // [m]      RADAR DETECTION CAPACITY
#define NEXTPOINT       0.5     // [m]      RESOLUTION OF WAYPOINT
#define FIRSTPOINT      3.0     // [m]
#define DIS_GENERATOR   0.3     // [m]      RESOLUTION OF LL GENERATOR
#define LIMT_BEARING    45      // [deg]    FOV OF LL GENERATOR
#define RESOL_BEARING   5       // [deg]
#define DIS_SAFE        2.0     // [m]      BOUNDARY OF OBSTACLES
#define obs_size        1000    //          THE NUMBER OF OBSTACLES


using namespace ros;
using namespace std;

float heading=0; // Local coordinate, heading=0
float goal[2];
float waypoint[60][2];
float point[9][2];
float obs[100][2];


void callback_goal(const std_msgs::Float32MultiArray &msg);
void callback_obs(const std_msgs::Float32MultiArray &msg);
void callback_heading(const std_msgs::Float32MultiArray &msg);

float point_generator(){

    float   pos_x = 0;    //local position_x of owncar
    float   pos_y = 0;    //local position_y of owncar
    int     num_bearing = int(2*(LIMT_BEARING/RESOL_BEARING)+1);  // must be int
    float   dist_pg_goal[num_bearing];
    int     path_count = 0; // initialize count for path
    int     path_complete = 0;

    while (path_complete==0){
        // LL Generator
        for(int i=0; i<num_bearing; i++ ){
            int check   = 0;
            int ang     = (heading - LIMT_BEARING + RESOL_BEARING * i) * D2R; // [rad]

            for(int ii=0; ii<10; ii++){
                float d1     = DIS_GENERATOR * ii;
                point[ii][0] = pos_x + (FIRSTPOINT + d1) * cos(ang); // local_x [m]
                point[ii][1] = pos_y + (FIRSTPOINT + d1) * sin(ang); // local_y [m]

                for(int k=0;k<100;k++){                 ////////////////////////////////////////// number of obstacles should be discussed
                    float d2    = sqrt((pos_x-obs[k][0])*(pos_x-obs[k][0]) + (pos_y-obs[k][1])*(pos_y-obs[k][1]));
                    float d3    = sqrt((point[ii][0]-obs[k][0])*(point[ii][0]-obs[k][0]) + (point[ii][1]-obs[k][1])*(point[ii][1]-obs[k][1]));
                    float theta = acos((d1*d1+d2*d2-d3*d3)/2*d1*d2); // law of 2nd cosine

                    if( d2<d1 ){
                        if(d2*sin(theta)<DIS_SAFE){
                            check = 1;
                            // safety check is required between obs and LL point

                            // calculate total distance including distance between start and waypoint, and the other distance between waypoint and goal
                            float dist_s_to_p   = sqrt((pos_x-point[ii][0])*(pos_x-point[ii][0]) + (pos_y-point[ii][1])*(pos_y-point[ii][1]));
                            float dist_p_to_g   = sqrt((point[ii][0]-goal[0])*(point[ii][0]-goal[0]) + (point[ii][1]-goal[1])*(point[ii][1]-goal[1]));
                            float dist_total    = dist_s_to_p + dist_p_to_g;
                            dist_pg_goal[i]     = dist_total;

                            break;
                        }
                    }
                }
            }
            if (check==1){
                break;
            }
        }

        // Find the shortest path to goal
        float min;	int ord;
        min	= dist_pg_goal[0];
        ord	= 0;

        for (i=1; i<K; i++)
        {
                if (distan[i]<min)
                {
                        min	= distan[i];    // update minimum value
                        ord	= i;            // update the order for minimum
                }
        }


        float dis_goal = sqrt(goal[0]*goal[0] + goal[1]*goal[1]);

        if (dis_goal < FIRSTPOINT)
        {
                waypoint[path_count][0] = goal[0];
                waypoint[path_count][1] = goal[1];
                path_complete = 1;
        }

        if (dis_goal >=FIRSTPOINT)
        {
                int ang     = (heading - LIMT_BEARING + RESOL_BEARING * i) * D2R; // [rad]

                pos_x	= pos_x + NEXTPOINT*cos(ang);
                pos_y	= pos_x + NEXTPOINT*sin(ang);

                path[path_count][0]	= pos_x;
                path[path_count][1]	= pos_x;
                path_count		+= 1;
        }
    }

    path_complete = 0;
}


int main(int argc, char** argv){
    // node name initialization
    init(argc, argv, "astar");

    // for debugging
    printf("Initiate: astar_node\n");

    //ros
    ros::NodeHandle nh;
    ros::Rate loop_rate(Hz);

    //Publisher & Subscribe
    Publisher  GPS_pos_pub   = nh.advertise<std_msgs::Float32MultiArray>("/astar_waypoint", 100);
    Subscriber goal_sub      = nh.subscribe("/goal",    1, callback_goal    );
    Subscriber heading_sub   = nh.subscribe("/heading", 1, callback_heading );
    Subscriber obs_sub       = nh.subscribe("/obs",     1, callback_obs     );

    while (ros::ok())
    {
        // Generate LL point and calculate distance
        point_generator();



        //waitKey(ROS_waitkey);  //50s
        ros::spinOnce();
        loop_rate.sleep();
    }
    return -1;
}

void callback_goal(const std_msgs::Float32MultiArray &msg){
    goal[0] 		 = msg.data[0];   // North      [m]
    goal[1] 		 = msg.data[1];   // East       [m]
}

void callback_heading(const std_msgs::Float32MultiArray &msg){
    heading 		 = msg.data[0];   // Heading [deg]
}

void callback_obs(const std_msgs::Float32MultiArray &msg){
    for(int i=0;i<obs_size; i++){
        // callback for multi obstacles
//        obs[i][0] 	 = msg.data[i][0];   // x [m]
//        obs[i][1] 	 = msg.data[i][1];   // y [m]
    }
}
