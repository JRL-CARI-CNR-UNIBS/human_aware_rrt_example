#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <human_probablistic_occupancy/human_probablistic_occupancy.h>

sensor_msgs::PointCloud pc;
geometry_msgs::PoseArray array;

bool updateOccupancyFcn(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{

  /* Select a random point of the pc as center of the "conic" distribution */
  int mu_idx = rand() % pc.points.size();
  std::vector<double> mu(3);
  mu.at(0)=pc.points.at(mu_idx).x;
  mu.at(1)=pc.points.at(mu_idx).y;
  mu.at(2)=pc.points.at(mu_idx).z;

  for (unsigned int idx=0;idx<pc.points.size();idx++)
  {
    std::vector<double> point(3);
    point.at(0) = pc.points.at(idx).x;
    point.at(1) = pc.points.at(idx).y;
    point.at(2) = pc.points.at(idx).z;

    double norm=0;
    for (unsigned int idx=0;idx<point.size();idx++)
      norm+=(point.at(idx)-mu.at(idx))*(point.at(idx)-mu.at(idx));
    norm=std::sqrt(norm);

    double r = 1;

    /* assign an occupancy probability from a linear function y=1-1/r*norm(point-x) */
    pc.channels.at(0).values.at(idx)=std::max(0.0,1-(1/r)*norm);

  }

  /* update poseArray */
  array.poses.clear();

  geometry_msgs::Pose pose;
  array.header.frame_id="world";
  pose.position.x=mu.at(0);
  pose.position.y=mu.at(1);
  pose.position.z=mu.at(2);

  array.poses.push_back(pose);

  ROS_INFO("Occupancy pointcloud updated");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_occupancy_node");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  ros::ServiceServer service = nh.advertiseService("update_occupancy", updateOccupancyFcn);
  ros::Publisher pc_pub=nh.advertise<sensor_msgs::PointCloud>("occupancy",1);
  ros::Publisher poses_pub=nh.advertise<geometry_msgs::PoseArray>("poses",1);



  Eigen::Vector3d x_min;
  Eigen::Vector3d x_max;
  x_min.setConstant(-3);
  x_max.setConstant(3);

  unsigned int npnt=50;
  human_occupancy::OccupancyGrid grid(x_min,x_max,npnt);


  std::vector<double> center(3);
  if (!nh.getParam("occ_center",center))
  {
    center.at(0)=1;
    center.at(1)=1;
    center.at(2)=1;
  }
  std::vector<double> radius(3,0.4);
  if (!nh.getParam("occ_radius",radius))
  {
    ROS_WARN("occ_radius not found");
  }

  for (double r=0;r<=1;r+=0.05)
  {
    for (double a1=0;a1<2*M_PI;a1+=0.05*M_PI)
    {
      for (double a2=0;a2<M_PI;a2+=0.05*M_PI)
      {
        geometry_msgs::Pose pose;
        pose.position.x=center.at(0)+r*radius.at(0)*std::sin(a1)*std::cos(a2);
        pose.position.y=center.at(1)+r*radius.at(1)*std::sin(a1)*std::sin(a2);
        pose.position.z=center.at(2)+r*radius.at(2)*std::cos(a1);
        array.poses.push_back(pose);
      }
    }
  }

  grid.update(array);
  pc=grid.toPointCloud();

  ROS_INFO("PointCloud Occupancy server ready.");


  while (ros::ok())
  {
    ros::spinOnce();

    pc_pub.publish(pc);
    poses_pub.publish(array);

    rate.sleep();
  }

  return 0;
}
