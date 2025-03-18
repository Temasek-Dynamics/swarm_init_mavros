#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
struct Quaternion {
    double w, x, y, z;

    // 计算四元数叉乘 (Hamilton 乘法)
    Quaternion operator*(const Quaternion& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,  // w'
            w * q.x + x * q.w + y * q.z - z * q.y,  // x'
            w * q.y - x * q.z + y * q.w + z * q.x,  // y'
            w * q.z + x * q.y - y * q.x + z * q.w   // z'
        };
    }

    void print() const {
        std::cout << "(" << w << ", " << x << ", " << y << ", " << z << ")\n";
    }
};

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
double x_odom_restart = 0;
double y_odom_restart = 0;
double z_odom_restart = 0;

geometry_msgs::PoseStamped swarm_init_offset;
double x_init_offset = 0;
double y_init_offset = 0;
double z_init_offset = 0;

double qx_init_offset = 0;
double qy_init_offset = 0;
double qz_init_offset = 0;
double qw_init_offset = 0;
Quaternion q_offset = {1, 0, 0, 0};
Quaternion q_odom = {1, 0, 0, 0};
Quaternion q_mavros = {1, 0, 0, 0};

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
            // y_odom_init_sum += Odom.pose.pose.position.x;
            // x_odom_init_sum += (-Odom.pose.pose.position.y);
            // z_odom_init_sum += Odom.pose.pose.position.z;
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


    x_odom_restart = Odom.pose.pose.position.x;
    y_odom_restart = Odom.pose.pose.position.y;
    z_odom_restart = Odom.pose.pose.position.z;

    // quaternion offset is the opposite of the current quaternion
    // the quaternion inverse
    qx_init_offset = -Odom.pose.pose.orientation.x;
    qy_init_offset = -Odom.pose.pose.orientation.y;
    qz_init_offset = -Odom.pose.pose.orientation.z;
    qw_init_offset =  Odom.pose.pose.orientation.w;
    
    q_offset = {qw_init_offset, qx_init_offset, qy_init_offset, qz_init_offset};

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
        
        q_offset = {1, 0, 0, 0};
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = Odom.header.stamp;
    pose.header.frame_id = "world"; // this is the frame_id of the world frame
    // pose.pose.position.x = Odom.pose.pose.position.x + x_init_offset - x_odom_init - x_odom_restart;
    // pose.pose.position.y = Odom.pose.pose.position.y + y_init_offset - y_odom_init - y_odom_restart;
    // pose.pose.position.z = Odom.pose.pose.position.z + z_init_offset - z_odom_init - z_odom_restart;
    pose.pose.position.x = Odom.pose.pose.position.x + x_init_offset  - x_odom_restart;
    pose.pose.position.y = Odom.pose.pose.position.y + y_init_offset  - y_odom_restart;
    pose.pose.position.z = Odom.pose.pose.position.z + z_init_offset  - z_odom_restart;

    q_odom = {Odom.pose.pose.orientation.w, Odom.pose.pose.orientation.x, Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z};

    q_mavros = q_offset * q_odom;//w,x,y,z
    pose.pose.orientation.x = q_mavros.x;
    pose.pose.orientation.y = q_mavros.y;
    pose.pose.orientation.z = q_mavros.z;
    pose.pose.orientation.w = q_mavros.w;
    swarm_pos_pub.publish(pose);
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