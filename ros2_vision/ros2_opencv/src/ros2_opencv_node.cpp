#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {


      SimpleBlobDetector::Params params;
        
      params.minThreshold = 50;
      params.maxThreshold = 200;
      params.filterByArea = false;
      params.filterByCircularity = true;
      params.minCircularity = 0.1;
      params.filterByConvexity = false;
      params.minConvexity = 0.87;
      params.filterByInertia = false;
      params.minInertiaRatio = 0.8;       


      detector_=SimpleBlobDetector::create(params);


    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "videocamera", 10, std::bind(&MinimalImagePublisher::topic_callback, this, _1));


    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("detection_image", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }
 
private:
  void timer_callback() {

        std::vector<KeyPoint> keypoints;
        
        detector_->detect( img_, keypoints);
        Mat im_with_keypoints;
        drawKeypoints( img_, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        //imshow("keypoints", im_with_keypoints );
        waitKey(1);
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();
                  
        publisher_->publish(*msg_.get());
        RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
        count_++;
  }

  void topic_callback (const sensor_msgs::msg::Image msg) 
    {
      
       RCLCPP_INFO(this->get_logger(), "Received Image:");
       img_ = cv_bridge::toCvCopy(msg,"bgr8")->image;
      
       waitKey(1);
    }


  Mat img_;
  Ptr<SimpleBlobDetector> detector_; 

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}