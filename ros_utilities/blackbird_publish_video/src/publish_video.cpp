#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

class republishImages {
public:
  republishImages(ros::NodeHandle nh, image_transport::ImageTransport it): _nh(nh), _it(it) {
    _nh.getParam("video_name", video_name);
    ROS_INFO("Video name: %s", video_name.c_str());

    std::string timestamp_file_name;
    _nh.getParam("timestamp_file_name", timestamp_file_name);
    ROS_INFO("Timestamp file: %s", timestamp_file_name.c_str());

    _nh.getParam("desired_rate", _desiredRate);
    ROS_INFO("Desired Rate: %f s", 1/_desiredRate);

    uint64_t stamp_ns;
    std::ifstream infile (timestamp_file_name);
    while (infile >> stamp_ns)
       timestamps.push_back(stamp_ns * 1e-9);

    _capPtr.reset(new cv::VideoCapture(video_name.c_str()));
    
    _pub = _it.advertise("/image_raw",1);
    
    _info_sub = _nh.subscribe("/camera_info", 1, &republishImages::cb_camera, this);
  };

  void cb_camera(const sensor_msgs::CameraInfo::ConstPtr& msg) {
     if ( (msg->header.stamp.toSec() - lastTimestamp) < (1/(_desiredRate+1)) )
        return;
     while (timestamps.at(_counter) != msg->header.stamp.toSec()) {
       *_capPtr >> _frame;
       _counter ++;
       if (_counter > timestamps.size())
	   throw std::runtime_error("Requested a timestamp not in the timestamps file");

     }
     if(!_frame.empty()) { 
       _msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg();
       _msg->header.stamp = msg->header.stamp;
       _pub.publish(_msg);
       lastTimestamp = msg->header.stamp.toSec();
     } 
     
  }
  
private:
  // Grab node handle to read the parameters
  ros::NodeHandle _nh;
  
  // Strings to store the video names
  std::string video_name;
  
  // Frame to grab the frame from the camera
  cv::Mat _frame;

  // Message Ptrs to populate the image messages
  sensor_msgs::ImagePtr _msg;

  // Video Capture ptr to grab the video streams
  std::unique_ptr<cv::VideoCapture> _capPtr;
  
  // Need an image transport ptr to publish images
  image_transport::ImageTransport _it;
  image_transport::Publisher _pub;

  // Subscribers for camera info subscribers
  ros::Subscriber _info_sub;

  // Vector of all the timestamps
  std::vector<double> timestamps;
  int _counter = 0;

  // Rate subsampling
  double _desiredRate = 119;
  double lastTimestamp;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  republishImages _republishImages(nh, it);

  ros::spin();
}

