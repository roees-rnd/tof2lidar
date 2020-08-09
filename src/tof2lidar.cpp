#include <stdio.h>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */

#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/LaserScan.h"
using namespace sensor_msgs;

#define DELTA_ANG_PER_RAY_DEG 1
#define TOF_HALF_FOV_DEG 10
#define DEG2RAD 0.017453293
#define NOISE_PER_DIST_METER 0.2
#define MAX_RANGE 2.5
#define USE_NORM_DIST

#ifdef USE_NORM_DIST
#include <math.h>
#endif

int32_t numRaysLS;
float_t angInc;
int start_inds[16] = {0};
int stop_inds[16] = {0};

float *noise_per_dist_meter_ptr;
float *max_range_ptr;

LaserScan msg_new;
ros::Publisher *pubPntr;

#ifdef USE_NORM_DIST
static float_t gauss(void)
{
    float_t x = (float_t)random() / RAND_MAX,
            y = (float_t)random() / RAND_MAX,
            z = sqrt(-2 * log(x)) * cos(2 * M_PI * y);
    return z;
}
#endif

class my_params
{
private:
    /* data */
public:
    float delta_ang_per_ray_deg;
    float tof_half_fov_deg;
    float noise_per_dist_meter;
    float max_range;
    std::string frame_out;
    bool flipped;
}prm;

int32_t rev_ind(int32_t index)
{
    return prm.flipped?numRaysLS-index-(int32_t)1:index;
}

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
            if (msg->ranges[i]<=*max_range_ptr){
            msg_new.ranges[rev_ind(tmp_ind_wrap)] = msg->ranges[i];
#ifdef USE_NORM_DIST
            msg_new.ranges[rev_ind(tmp_ind_wrap)] += (float)(msg->ranges[i]) * (*noise_per_dist_meter_ptr) * gauss();
#elif
            msg_new.ranges[rev_ind(tmp_ind_wrap)] += (float)(msg->ranges[i]) * (*noise_per_dist_meter_ptr) * (float)rand() / RAND_MAX;
#endif
            }else{
                msg_new.ranges[rev_ind(tmp_ind_wrap)]=0;
            }
            if (tmp_ind == stop_inds[i])
            {
                break;
            }
            tmp_ind++;
        }
    }
    //std::reverse(msg_new.ranges.begin(),msg_new.ranges.end());
    pubPntr->publish(msg_new);
    return;
}

void load_params(ros::NodeHandle &n, my_params& prm);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tof2lidar_node");
    ros::NodeHandle n;
    std::string s;

    LaserScan msg = LaserScan();

    ROS_WARN("node name=%s",ros::this_node::getName().c_str());
    ROS_WARN("node namespace=%s",ros::this_node::getNamespace().c_str());
    
    // std::cout << "---------------- nh.name=" << s.c_str() << std::endl;
    // my_params prm;
    noise_per_dist_meter_ptr = &prm.noise_per_dist_meter;
    max_range_ptr = &prm.max_range;
    load_params(n, prm);


    

    numRaysLS = (int)(360.0 / ((float)prm.delta_ang_per_ray_deg));
    angInc = 360.0 / (float_t)(numRaysLS);

    msg_new.angle_increment = angInc * DEG2RAD;
    msg_new.angle_min = 0;
    msg_new.angle_max = (numRaysLS - 1) * angInc * DEG2RAD;
    msg_new.header.frame_id = prm.frame_out;
    msg_new.range_max = 4.0;
    msg_new.range_min = 0.01;
    msg_new.scan_time = 0.0;
    msg_new.time_increment = 0.0;
    std::vector<float_t> ranges(numRaysLS, 0);
    msg_new.ranges = ranges;
    std::vector<float_t> intensities(numRaysLS, 0);
    msg_new.intensities = intensities;
    srand(time(NULL));

    int i_tmp = 0;
    for (int i = 0; i < 16; i++)
    {
        i_tmp = (22.5 * i - prm.tof_half_fov_deg) / prm.delta_ang_per_ray_deg;
        if ((int)i_tmp == (int)(i_tmp + 0.5))
        {
            start_inds[i] = (int)i_tmp;
        }
        else
        {
            start_inds[i] = (int)i_tmp + 1;
        }
        i_tmp = (22.5 * i + prm.tof_half_fov_deg) / prm.delta_ang_per_ray_deg;
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


void load_params(ros::NodeHandle &n, my_params& prm)
{
    if (n.param("tof2lidar/delta_ang_per_ray_deg", prm.delta_ang_per_ray_deg, (float)DELTA_ANG_PER_RAY_DEG))
    {
        ROS_INFO("GOT PARAM: delta ang per ray = %f", prm.delta_ang_per_ray_deg);
    }
    else
    {
        ROS_INFO("NO  PARAM: delta ang per ray = %f", (float)DELTA_ANG_PER_RAY_DEG);
    }

    if (n.param("tof2lidar/tof_half_fov_deg", prm.tof_half_fov_deg, (float)TOF_HALF_FOV_DEG))
    {
        ROS_INFO("GOT PARAM: tof_half_fov_deg = %f", prm.tof_half_fov_deg);
    }
    else
    {
        ROS_INFO("NO  PARAM: tof_half_fov_deg = %f", (float)TOF_HALF_FOV_DEG);
    }

    if (n.param("tof2lidar/noise_per_dist_meter", prm.noise_per_dist_meter, (float)NOISE_PER_DIST_METER))
    {
        ROS_INFO("GOT PARAM: noise_per_dist_meter = %f", prm.noise_per_dist_meter);
    }
    else
    {
        ROS_INFO("NO  PARAM: noise_per_dist_meter = %f", (float)NOISE_PER_DIST_METER);
    }

    if (n.param("tof2lidar/max_range", prm.max_range, (float)MAX_RANGE))
    {
        ROS_INFO("GOT PARAM: max_range = %f", prm.max_range);
    }
    else
    {
        ROS_INFO("NO  PARAM: max_range = %f", (float)MAX_RANGE);
    }

    if (n.param<std::string>("tof2lidar/frame_out", prm.frame_out, (std::string)"mr18_frame"))
    {
        ROS_INFO("GOT PARAM: frame_out = %s", prm.frame_out.c_str());
    }
    else
    {
        ROS_INFO("NO  PARAM: frame_out = %s", "mr18_frame");
    }

    if (n.param<bool>("tof2lidar/flipped", prm.flipped, false))
    {
        ROS_INFO("GOT PARAM: flipped = %s", prm.flipped?"true":"false");
    }
    else
    {
        ROS_INFO("NO  PARAM: flipped = \"false\"");
    }
}