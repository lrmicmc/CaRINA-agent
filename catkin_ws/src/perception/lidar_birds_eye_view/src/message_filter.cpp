#include "ros/ros.h"

#include <msgs_perception/ObstacleArray.h>
#include <msgs_perception/Obstacle.h>
// #include <tf/transform_datatypes.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>

#include "geometry_msgs/PointStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseDrawer
{
public:
  PoseDrawer() :
    tf2_(buffer_),  target_frame_("base_link"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "/lidar1/lidar_obstacles_array", 10);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
    ROS_INFO("class" );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  // void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr) 
  void msgCallback(const msgs_perception::ObstacleArrayConstPtr& obstaclesGT)
  {
    msgs_perception::ObstacleArray::ConstPtr radarDataArray_;
    ROS_INFO("callback" );
    std::cout << "I received "<< obstaclesGT->obstacle[0].pose.position.x << std::endl;
    radarDataArray_ = obstaclesGT;

    cv::Mat bev_lidar_b_gt(500, 1000, CV_8UC3, cv::Scalar(0));

    for (int j = 0; j < radarDataArray_->obstacle.size(); j++) 
    //         if (radarDataArray_->obstacle[j].pose.position.x != 0 and radarDataArray_->obstacle[j].pose.position.y != 0) {
    // BOOST_FOREACH (const msgs_perception::Obstacle& obstacle_n, radarDataArray_->obstacle)
    { 
      geometry_msgs::PointStamped point_in;
      geometry_msgs::PointStamped point_out;

      point_in.header.frame_id = radarDataArray_->obstacle[j].header.frame_id ;
      point_in.header.stamp =    radarDataArray_->obstacle[j].header.stamp;// ;ros::Time();
      point_in.point.x = radarDataArray_->obstacle[j].pose.position.x;
      point_in.point.y = radarDataArray_->obstacle[j].pose.position.y;
      point_in.point.z = radarDataArray_->obstacle[j].pose.position.z;

      try 
      {
        buffer_.transform(point_in, point_out, target_frame_);
        
        ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
               point_out.point.x,
               point_out.point.y,
               point_out.point.z);
      }
      catch (tf2::TransformException &ex) 
      {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
      }
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  // message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  message_filters::Subscriber<msgs_perception::ObstacleArray> point_sub_;
  tf2_ros::MessageFilter<msgs_perception::ObstacleArray> tf2_filter_;

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "pose_drawer"); //Init ROS
  PoseDrawer pd; //Construct class
  ROS_INFO("ini" );
  ros::spin(); // Run until interupted 
  return 0;
};