#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <msgs_perception/ObstacleArray.h>
#include <msgs_perception/Obstacle.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  msgs_perception::ObstacleArray::ConstPtr radarDataArray_;
  double map_resolution;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub_radar;


  // image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_map;
  image_transport::Publisher image_pub_map;
  image_transport::Publisher image_pub_pc_gray;
  image_transport::Publisher image_pub_gt;




public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    sub = nh_.subscribe<PointCloud>("/lidar1", 1, &ImageConverter::lidarCallback, this);
    sub_radar = nh_.subscribe("/carina/perception/lidar/gt_obstacle_array", 1,  &ImageConverter::callbackRadar, this);
	sub_map = it_.subscribe("/image_patch_map", 1, &ImageConverter::mapImageCallback, this);
  
    // image_pub_= it_.advertise("/image_converter/output_video", 1);
    image_pub_map = it_.advertise("/resized_map", 1);
    image_pub_pc_gray = it_.advertise("/pointcloud_image_gray", 1);
    image_pub_gt = it_.advertise("/image_pointcloud_gt", 1);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }




  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }


  void callbackRadar(const msgs_perception::ObstacleArray::ConstPtr& obstaclesGT) 
  {
  	  	double map_resolution = 5.0;

  radarDataArray_ = obstaclesGT;

  cv::Mat bev_lidar_b_gt(568*2, 568*2, CV_8UC3, cv::Scalar(0,0,0));
  cv::Mat bev_lidar_f_gt(568*2, 568*2, CV_8UC3, cv::Scalar(0,0,0)); 

  std::ostringstream oss;
  oss << radarDataArray_->obstacle[0].header.stamp;
  std::string numString = oss.str();
  boost::erase_all(numString, ".");
  int classes_id = 0;
	  for (int j = 0; j < radarDataArray_->obstacle.size(); j++) 
	  { 

	    double x_point = radarDataArray_->obstacle[j].pose.position.y;
	    double y_point = radarDataArray_->obstacle[j].pose.position.x;
	    // double z = radarDataArray_->obstacle[j].pose.position.z;
	    double w = radarDataArray_->obstacle[j].scale.y * map_resolution;
	    double h = radarDataArray_->obstacle[j].scale.x * map_resolution;
	    double obj_yaw = tf::getYaw(radarDataArray_->obstacle[j].pose.orientation);

	    double yaw_degrees = -(obj_yaw * 180.0 / M_PI); // conversion to degrees

	    std::string classe_name = radarDataArray_->obstacle[j].classes[0];

	    if (classe_name == "car"){
	      classes_id=1;
	    }

	    if (classe_name == "motorcycle"){
	      classes_id=2;
	    }

	    if (classe_name == "bus"){
	      classes_id=3;
	    }

	    if (classe_name == "bicycle"){
	      classes_id=4;
	    }

	    if (classe_name == "truck"){
	      classes_id=5;
	    }

	    if (classe_name == "pedestrian"){
	      classes_id=6;
	    }

	    if (classe_name == "other_vehicle"){
	      classes_id=7;
	    }

	    if (classe_name == "animal"){
	      classes_id=8;
	    }

	    if (classe_name == "emergency_vehicle"){
	      classes_id=9;
	    }

	      int px = 568+(int)(x_point*map_resolution);
	      int py = 568+(int)(y_point*map_resolution);
	      cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(px,py), 
	      cv::Size2f(w,h), yaw_degrees);
	      cv::Point2f vertices2f[4];
	      rRect.points(vertices2f);
	      cv::Point vertices[4];
	      for (int i = 0; i < 4; i++)
	          vertices[i] = vertices2f[i];

	      cv::fillConvexPoly(bev_lidar_f_gt, vertices, 4, cv::Scalar(0,0,classes_id));
	  }
	  sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bev_lidar_f_gt).toImageMsg();
	  image_pub_gt.publish(msg_out);
	  // cv::imshow("lidar", bev_lidar_f_gt);
   // 	  // cv::imshow("imageMap", cv_bridge::toCvShare(msg, "bgr8")->image);
   //    cv::waitKey(100);    
  
  // numString.erase (numString.begin()+16, numString.end());
}

void lidarCallback(const PointCloud::ConstPtr& pcl_in)
{
	  	double map_resolution = 5.0;

   std::ostringstream oss;
   oss << pcl_in->header.stamp;
   std::string numString = oss.str();
   boost::erase_all(numString, ".");
 
  cv::Mat bev_lidar_b(568*2,  568*2, CV_8UC1, cv::Scalar(0));
  cv::Mat bev_lidar_f(568*2,  568*2, CV_8UC1, cv::Scalar(0));

  BOOST_FOREACH (const pcl::PointXYZ& pt, pcl_in->points)
  {
    double x_point = pt.y;
    double y_point = pt.x;
    float z_height = pt.z;

    double v =  (255.0f*(z_height-(-5.0f) )/((4.0f)-(-5.0f)));
    float vmin = 0.0f;
    float vmax = 255.0f;
    float dv = vmax - vmin;
    float r = 255.0f, g = 255.0f, b = 255.0f;

    if (v < (vmin + 0.25f * dv)) 
    {
        r = 0.0f;
        g = 255.0f*(4.0f * (v - vmin) / dv);
    } else if (v < (vmin + 0.5f * dv)) {
        r = 0.0f;
        b = 255.0f*(1.0f + 4.0f * (vmin + 0.25f * dv - v) / dv);
    } else if (v < (vmin + 0.75f * dv)) {
        r = 255.0f*(4.0f * (v - vmin - 0.5f * dv) / dv);
        b = 0.0f;
    } else {
        g = 255.0f*(1.0f + 4.0f * (vmin + 0.75f * dv - v) / dv);
        b = 0.0f;
    }

      int px = 568+(int)(x_point*map_resolution);
      int py = 568+(int)(y_point*map_resolution);

      circle( bev_lidar_f, cv::Point(px, py),  0, cv::Scalar( v ), -1, 8 );
    }
   	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", bev_lidar_f).toImageMsg();
   	image_pub_pc_gray.publish(msg_out);
 
}

	void mapImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		  	double map_resolution = 5.0;

	  cv::Mat map_image_resize(568*2, 568*2, CV_8UC3, cv::Scalar(0,0,0));
	  std::ostringstream oss;
	  oss << msg->header.stamp;
	  std::string numString = oss.str();
	  boost::erase_all(numString, ".");
	  try
	  {

	    cv::Mat map_image = cv_bridge::toCvShare(msg, "bgr8")->image;
	    cv::resize(map_image, map_image_resize, map_image_resize.size(), 0, 0);
	  }
	  catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
	  sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_image_resize).toImageMsg();
	  // image_pub_.publish(map_image_resize->toImageMsg());
	  image_pub_map.publish(msg_out);

	}


};






int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ImageConverter ic;
  ros::spin();
  return 0;
}


