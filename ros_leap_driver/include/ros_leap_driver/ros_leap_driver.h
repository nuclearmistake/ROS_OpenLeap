#ifndef _OPENLEAP
#define _OPENLEAP

#include <low-level-leap.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <queue>
#include <stack>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

int main(int argc, char ** argv);

#endif
