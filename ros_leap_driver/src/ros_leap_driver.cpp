#include <ros_leap_driver/ros_leap_driver.h>

leap::driver *drv;

using namespace std;

class frame_t
{
public:
  frame_t() : left(sensor_msgs::ImagePtr(new sensor_msgs::Image())), right(sensor_msgs::ImagePtr(new sensor_msgs::Image())) { }  
  sensor_msgs::ImagePtr left;
  sensor_msgs::ImagePtr right;
};

frame_t *current;

image_transport::ImageTransport *lefttrans, *righttrans;
image_transport::CameraPublisher leftpub, rightpub;
std::string left_frame, right_frame;
camera_info_manager::CameraInfoManager *leftmgr, *rightmgr;

void publishframe(camera_info_manager::CameraInfoManager *mgr, const image_transport::CameraPublisher &campub, const sensor_msgs::ImagePtr img, const std::string& frame)
{
  ros::Time timestamp = ros::Time::now();
  sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mgr->getCameraInfo()));
  std_msgs::Header::Ptr imageheader(new std_msgs::Header());
  std_msgs::Header::Ptr cinfoheader(new std_msgs::Header());
  imageheader->frame_id = frame;
  cinfoheader->frame_id = frame;
  imageheader->stamp = timestamp;
  cinfoheader->stamp = timestamp;
  img->header = *imageheader;
  cinfo->header = *cinfoheader;
  campub.publish(img, cinfo);
}

void gotData(camdata_t *data)
{
  if (current == NULL)
  {
	    current = new frame_t();
      current->left->width = current->right->width = VFRAME_WIDTH;
      current->left->height = current->right->height = VFRAME_HEIGHT*2;
      current->left->encoding = current->right->encoding = "rgb8";
      current->left->step = current->right->step = current->left->width * 3;
//      current->left->data.resize(VFRAME_SIZE);
//      current->right->data.resize(VFRAME_SIZE);
  }

  current->left->data.clear();
  current->right->data.clear();
  for(int y=0;y<VFRAME_HEIGHT;y++)
  {
    for(int tworows=0;tworows<2;tworows++)
      for (int x=0; x<VFRAME_WIDTH; x++)
      {
        current->left->data.push_back(data->left[VFRAME_WIDTH * y + x]);
        current->left->data.push_back(data->left[VFRAME_WIDTH * y + x]);
        current->left->data.push_back(data->left[VFRAME_WIDTH * y + x]);
        current->right->data.push_back(data->right[VFRAME_WIDTH * y + x]);
        current->right->data.push_back(data->right[VFRAME_WIDTH * y + x]);
        current->right->data.push_back(data->right[VFRAME_WIDTH * y + x]);
      }
  }

//  memcpy(&current->left->data[0], data->left, VFRAME_SIZE);
//  memcpy(&current->right->data[0], data->right, VFRAME_SIZE);

  const sensor_msgs::ImagePtr l=(const sensor_msgs::ImagePtr)current->left, r=(const sensor_msgs::ImagePtr)current->right;
  publishframe(leftmgr, leftpub, l, left_frame);
  publishframe(rightmgr, rightpub, r, right_frame);
}

// leap init, loop spinner, and leap deinit
void doit()
{
  drv = new leap::driver(&gotData);
//  drv->agressivelyAvoidFlashes(true);
  drv->spin();
  if (current)
  {
    free(current);
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "");
	ros::NodeHandle nh;
	std::string leftcam, rightcam;
	if (!ros::param::get("~left_camera", leftcam))
	{
		leftcam = "/leap/left";
	}
	if (!ros::param::get("~right_camera", rightcam))
	{
		rightcam = "/leap/right";
	}
  ros::NodeHandle lnh(leftcam);
  ros::NodeHandle rnh(rightcam);
  leftmgr = new camera_info_manager::CameraInfoManager(lnh);
  leftmgr->setCameraName("leap_left");
  rightmgr = new camera_info_manager::CameraInfoManager(rnh);
  rightmgr->setCameraName("leap_right");
  left_frame = right_frame = "/leap";
  lefttrans = new image_transport::ImageTransport(lnh);
  leftpub = lefttrans->advertiseCamera("image_raw", 1, false);
  righttrans = new image_transport::ImageTransport(rnh);
  rightpub = righttrans->advertiseCamera("image_raw", 1, false);
  boost::thread spinner(doit);
  ros::spin();
  drv->shutdown();
  spinner.join();
  return (0);
}
