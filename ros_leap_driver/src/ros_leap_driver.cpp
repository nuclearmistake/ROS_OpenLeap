#include <ros_leap_driver/ros_leap_driver.h>
#define USE_MATH_DEFINES
#include <math.h>

using namespace std;

typedef struct ctx_s ctx_t;
struct ctx_s
{
  int quit;
};

class frame_t
{
public:
  frame_t() : left(sensor_msgs::ImagePtr(new sensor_msgs::Image())), right(sensor_msgs::ImagePtr(new sensor_msgs::Image())) { }  
  sensor_msgs::ImagePtr left;
  sensor_msgs::ImagePtr right;
  uint32_t id;
  uint32_t data_len;
  uint32_t state;
};

ctx_t ctx_data;
ctx_t *ctx = NULL;
map<uint32_t, frame_t*> frame;
vector<uint32_t> pending;
stack<frame_t *> recycling;
frame_t *current = NULL;

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

void process_video_frame(ctx_t *ctx)
{
  const sensor_msgs::ImagePtr l=(const sensor_msgs::ImagePtr)current->left, r=(const sensor_msgs::ImagePtr)current->right;
  publishframe(leftmgr, leftpub, l, left_frame);
  publishframe(rightmgr, rightpub, r, right_frame);
}

void process_usb_frame(ctx_t *ctx, unsigned char *data, int size)
{
  int i,x,y;

  int bHeaderLen = data[0];
  int bmHeaderInfo = data[1];

  uint32_t dwPresentationTime = *( (uint32_t *) &data[2] );

  frame_t* f = NULL;

  for(vector<uint32_t>::const_iterator it = pending.begin(); it!=pending.end();it++)
          if ((uint32_t)(*it) == dwPresentationTime)
          {
                  f = frame[dwPresentationTime];
                  break;
          }
  if (f == NULL)
  {
      if (!recycling.empty()) {
        f = recycling.top();
        recycling.pop();
        f->data_len = 0;
        f->state = 0;
      } else {
        f = new frame_t();
        f->left->width = f->right->width = VFRAME_WIDTH;
        f->left->height = f->right->height = VFRAME_HEIGHT;
        f->left->encoding = f->right->encoding = "rgb8";
        f->left->step = f->right->step = f->left->width * 3;
      }
      f->left->data.clear();
      f->right->data.clear();
      frame[dwPresentationTime] = f;
      pending.push_back(dwPresentationTime);
      sort(pending.begin(),pending.end());
      f->id = dwPresentationTime;
  }
  
  //printf("frame time: %u\n", dwPresentationTime);

  for (x=0,y=0,i=bHeaderLen; i < size; i += 2) {
    f->left->data.push_back(data[i]);
    f->left->data.push_back(data[i]);
    f->left->data.push_back(data[i]);
    f->right->data.push_back(data[i+1]);
    f->right->data.push_back(data[i+1]);
    f->right->data.push_back(data[i+1]);
    f->data_len++;
  }

  if (bmHeaderInfo & UVC_STREAM_EOF) {
    if (f->data_len != VFRAME_SIZE) {
      ROS_DEBUG("wrong frame size got %i expected %i", f->data_len, VFRAME_SIZE);
      recycling.push(f);
      frame.erase(f->id);
      pending.erase(remove(pending.begin(), pending.end(), f->id), pending.end());
      return ;
    }

    if (current!=NULL) {
      recycling.push(current);
    }
    current = frame[dwPresentationTime];
    pending.erase(remove(pending.begin(), pending.end(), dwPresentationTime), pending.end());
    frame.erase(dwPresentationTime);
    sort(pending.begin(),pending.end());
    if (pending.size() > 10) {
        ROS_WARN("Oh noez! There are %d pending frames in the queue!",(int)pending.size());
        for(vector<uint32_t>::const_iterator it = pending.begin(); it!=pending.begin()+5;it++)
        {
          recycling.push(frame[*it]);
          frame.erase(*it);
        }
        reverse(pending.begin(), pending.end());
        pending.resize(5);
        reverse(pending.begin(), pending.end());
    }

    process_video_frame(ctx);
  }
}

void gotData(unsigned char* data, int usb_frame_size)
{
  process_usb_frame(ctx, data, usb_frame_size);
}

// leap init, loop spinner, and leap deinit
void doit()
{
  memset(&ctx_data, 0, sizeof (ctx_data));
  ctx = &ctx_data;
  init();
  setDataCallback(&gotData);
  spin();
  if (current)
  {
    //free storage in frame
    free(current);
  }
  for(std::vector<uint32_t>::const_iterator it = pending.begin(); it != pending.end(); it++)
  {
        frame_t *tmp = frame[*it];
        //free storage in frame
        frame.erase((uint32_t)*it);
        free(tmp);
  }
  while(!recycling.empty()) {
    frame_t *tmp = recycling.top();
    recycling.pop();
    //free storage in frame
    free(tmp);
  }
}

int main(int argc, char **argv)
{
  for(int i=0;i<argc;i++)
		ROS_INFO("ARG[%d]=%s",i, argv[i]);
	ros::init(argc, argv, "");
	ros::NodeHandle nh;
	std::string left_topic, right_topic;	
	if (!ros::param::get("~left_topic", left_topic))
	{
		left_topic = "/leap/left/image_raw";
	}
	if (!ros::param::get("~right_topic", right_topic))
	{
		right_topic = "/leap/right/image_color";
	}
  if (!ros::param::get("~left_frame", left_frame))
	{
		left_frame = "/leap_left";
	}
	if (!ros::param::get("~right_frame", right_frame))
	{
		right_frame = "/leap_right";
	}
  ROS_INFO("PARMS: left_t:%s right_t:%s left_f:%s right_f:%s", left_topic.c_str(), right_topic.c_str(), left_frame.c_str(), right_frame.c_str());
  leftmgr = new camera_info_manager::CameraInfoManager(nh);
  leftmgr->setCameraName("leap_left");
  rightmgr = new camera_info_manager::CameraInfoManager(nh);
  rightmgr->setCameraName("leap_right");
  lefttrans = new image_transport::ImageTransport(nh);
  leftpub = lefttrans->advertiseCamera("left/image", 1, false);
  righttrans = new image_transport::ImageTransport(nh);
  rightpub = lefttrans->advertiseCamera("right/image", 1, false);
  boost::thread spinner(doit);
  ros::spin();
  shutdown();
  spinner.join();
  return (0);
}
