#include <stdio.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>       /* time */

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/LaserScan.h"
using namespace sensor_msgs;

#define DELTA_ANG_PER_RAY_DEG 1
#define TOF_HALF_FOV_DEG 10
#define DEG2RAD 0.017453293
#define NOISE_PER_DIST_METER 0.2

int32_t numRaysLS;
float_t angInc;
int start_inds[16] = {0};
int stop_inds[16] = {0};

LaserScan msg_new;
ros::Publisher *pubPntr;

void chatterCallback(const LaserScan::ConstPtr &msg)
{
    int tmp_ind = 0;
    int tmp_ind_wrap = 0;
    msg_new.header.seq = msg->header.seq;
    msg_new.header.stamp.nsec = msg->header.stamp.nsec;
    msg_new.header.stamp.sec = msg->header.stamp.sec;
    msg_new.ranges.assign(numRaysLS, 0);

    for (int i = 0; i < 16; i++)
    {
        tmp_ind = start_inds[i];
        while (true)
        {
            tmp_ind_wrap = tmp_ind >= 0 ? tmp_ind : tmp_ind + numRaysLS;
            msg_new.ranges[tmp_ind_wrap] = msg->ranges[i] + (float)(msg->ranges[i])*NOISE_PER_DIST_METER*(float)rand()/RAND_MAX; // add noise
            if (tmp_ind == stop_inds[i])
            {
                break;
            }
            tmp_ind++;
        }
    }
    pubPntr->publish(msg_new);
    return;
}

void load_params(ros::NodeHandle& n, int32_t& delta_ang_per_ray_deg, float& tof_half_fov_deg){
    if (n.param("delta_ang_per_ray_deg", delta_ang_per_ray_deg, (int32_t)DELTA_ANG_PER_RAY_DEG))
    {
        ROS_INFO("GOT PARAM: delta ang per ray = %d", delta_ang_per_ray_deg);
    }
    else
    {
        ROS_INFO("NO  PARAM: delta ang per ray = %d", DELTA_ANG_PER_RAY_DEG);
    }

    if (n.param("tof_half_fov_deg", tof_half_fov_deg, (float)TOF_HALF_FOV_DEG))
    {
        ROS_INFO("GOT PARAM: tof_half_fov_deg = %f", tof_half_fov_deg);
    }
    else
    {
        ROS_INFO("NO  PARAM: tof_half_fov_deg = %f", (float)TOF_HALF_FOV_DEG);
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tof2lidar_node");
    ros::NodeHandle n;

    LaserScan msg = LaserScan();
    int32_t delta_ang_per_ray_deg;
    float tof_half_fov_deg;
    load_params(n, delta_ang_per_ray_deg, tof_half_fov_deg);
    
    numRaysLS = (int)(360.0 / ((float)delta_ang_per_ray_deg));
    angInc = 360.0 / (float_t)(numRaysLS);

    msg_new.angle_increment = angInc * DEG2RAD;
    msg_new.angle_min = 0;
    msg_new.angle_max = (numRaysLS - 1) * angInc * DEG2RAD;
    msg_new.header.frame_id = "mr18_frame";
    msg_new.range_max = 4.0;
    msg_new.range_min = 0.01;
    msg_new.scan_time = 0.0;
    msg_new.time_increment = 0.0;
    std::vector<float_t> ranges(numRaysLS, 0);
    msg_new.ranges = ranges;
    std::vector<float_t> intensities(numRaysLS, 0);
    msg_new.intensities = intensities;
    srand (time(NULL));

    int i_tmp = 0;
    for (int i = 0; i < 16; i++)
    {
        i_tmp = (22.5 * i - tof_half_fov_deg) / delta_ang_per_ray_deg;
        if ((int)i_tmp == (int)(i_tmp + 0.5))
        {
            start_inds[i] = (int)i_tmp;
        }
        else
        {
            start_inds[i] = (int)i_tmp + 1;
        }
        i_tmp = (22.5 * i + tof_half_fov_deg) / delta_ang_per_ray_deg;
        if ((int)i_tmp == (int)(i_tmp + 0.5))
        {
            stop_inds[i] = (int)i_tmp;
        }
        else
        {
            stop_inds[i] = (int)i_tmp - 1;
        }
        ROS_INFO("start\\end_inds[%d]=%d,%d", i, start_inds[i] < 0 ? start_inds[i] + numRaysLS : start_inds[i], stop_inds[i]);
    }

    ros::Publisher chatter_pub = n.advertise<LaserScan>("mr18_scan", 10);
    pubPntr = &chatter_pub;
    ros::Subscriber sub = n.subscribe("mr18", 10, chatterCallback);

    ROS_INFO("mr18_scan number of rays=%d", numRaysLS);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
