#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry Odom;
ros::Publisher swarm_pos_pub;
bool is_swarm = true;
bool odom_received = false;
bool odom_init = false;
bool swarm_init_offset_received = false;
int odom_count = 0;
double x_odom_init_sum = 0;
double y_odom_init_sum = 0;
double z_odom_init_sum = 0;
double x_odom_init = 0;
double y_odom_init = 0;
double z_odom_init = 0;

geometry_msgs::PoseStamped swarm_init_offset;
double x_init_offset = 0;
double y_init_offset = 0;
double z_init_offset = 0;

void Odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    static bool first_log = true;

    if (first_log)
    {
        ROS_INFO("Odom received");
        first_log = false;
    }

    odom_received = true;
    Odom = *msg;

    if (!odom_init)
    {
        if (odom_count < 100)
        {
            x_odom_init_sum += Odom.pose.pose.position.x;
            y_odom_init_sum += Odom.pose.pose.position.y;
            z_odom_init_sum += Odom.pose.pose.position.z;
            odom_count++;
        }

        if (odom_count == 100)  // 仅在收集满 100 次数据时计算
        {
            x_odom_init = x_odom_init_sum / 100;
            y_odom_init = y_odom_init_sum / 100;
            z_odom_init = z_odom_init_sum / 100;
            odom_init = true;

            ROS_INFO("Odom init completed:");
            ROS_INFO("x_odom_init: %f", x_odom_init);
            ROS_INFO("y_odom_init: %f", y_odom_init);
            ROS_INFO("z_odom_init: %f", z_odom_init);
        }
    }
}

void SwarmInitOffset_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("Swarm init offset received");
    swarm_init_offset_received = true;
    swarm_init_offset = *msg;
    x_init_offset = swarm_init_offset.pose.position.x;
    y_init_offset = swarm_init_offset.pose.position.y;
    z_init_offset = swarm_init_offset.pose.position.z;
    ROS_INFO("x_init_offset: %f", x_init_offset);
    ROS_INFO("y_init_offset: %f", y_init_offset);
    ROS_INFO("z_init_offset: %f", z_init_offset);
}

void PubSwarmPosetoMavros()
{
    // if (!odom_received || !swarm_init_offset_received) return;
    if (!odom_received) return;
    else if (odom_received && !swarm_init_offset_received)
    {
        /* pub with no offset */
        x_init_offset = 0;
        y_init_offset = 0;
        z_init_offset = 0;
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = Odom.header.stamp;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = Odom.pose.pose.position.x + x_init_offset - x_odom_init;
    pose.pose.position.y = Odom.pose.pose.position.y + y_init_offset - y_odom_init;
    pose.pose.position.z = Odom.pose.pose.position.z + z_init_offset - z_odom_init;
    pose.pose.orientation.x = Odom.pose.pose.orientation.x;
    pose.pose.orientation.y = Odom.pose.pose.orientation.y;
    pose.pose.orientation.z = Odom.pose.pose.orientation.z;
    pose.pose.orientation.w = Odom.pose.pose.orientation.w;
    swarm_pos_pub.publish(pose);

    // set the init_offset to false after the first publish
    if (swarm_init_offset_received)
    {
        swarm_init_offset_received = false;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Odom_to_mavros");
  ros::NodeHandle nh("~");

  nh.param("is_swarm", is_swarm, bool (true));

  ROS_INFO("is_swarm: %d", is_swarm);
  
  if(!is_swarm)
  {
    swarm_init_offset_received = true;
    swarm_init_offset.pose.position.x = 0;
    swarm_init_offset.pose.position.y = 0;
    swarm_init_offset.pose.position.z = 0;  
  }


  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry> ("/Odometry", 100, Odom_cb);
  ros::Subscriber swarm_init_offset_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/initpose", 100, SwarmInitOffset_cb);
  swarm_pos_pub=nh.advertise<geometry_msgs::PoseStamped> ("/mavros/vision_pose/pose",10);

  ros::Rate rate(10.0);
  ROS_INFO("Odom to mavros node started");
  while (ros::ok())
  {
    PubSwarmPosetoMavros(); 
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}