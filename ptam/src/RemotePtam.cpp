/*

Copyright (c) 2008, Willow Garage, Inc.

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#ifdef HAVE_GTK
#include <gtk/gtk.h>

// Platform-specific workaround for #3026: image_view doesn't close when
// closing image window. On platforms using GTK+ we connect this to the
// window's "destroy" event so that image_view exits.
static void destroyNode(GtkWidget *widget, gpointer data)
{
  ros::shutdown();
}

#endif

class RemotePtam
{
private:
  image_transport::Subscriber *sub_;
  std::string window_name_;
  std::string transport_;
  std::string topic_;

public:
  RemotePtam(const ros::NodeHandle& nh, const std::string& _transport)
  {
    topic_ = "vslam/preview";
    ros::NodeHandle local_nh("~");
    local_nh.param("window_name", window_name_, topic_);

    transport_ = _transport;

    bool autosize;
    local_nh.param("autosize", autosize, false);
    cv::namedWindow(window_name_, autosize ? 1 : 0);

#ifdef HAVE_GTK
    // Register appropriate handler for when user closes the display window
    GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
#endif

    sub_ = NULL;
    subscribe(nh);
  }

  ~RemotePtam()
  {
    unsubscribe();
    cv::destroyWindow(window_name_);
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow(window_name_, cv_ptr->image);
  }
  void subscribe(const ros::NodeHandle& nh)
  {
    if (sub_ == NULL)
    {
      image_transport::ImageTransport it(nh);
      sub_ = new image_transport::Subscriber(it.subscribe(topic_, 1, &RemotePtam::image_cb, this, transport_));
    }
  }

  void unsubscribe()
  {
    if (sub_ != NULL)
    {
      delete sub_;
      sub_ = NULL;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vslam_remote", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  RemotePtam remote(n, (argc > 1) ? argv[1] : "raw");

  char key = 0;

  remote.subscribe(n);
  bool subscribed = true;

  ros::Publisher key_pub = n.advertise<std_msgs::String> ("vslam/key_pressed", 10);
  std_msgs::StringPtr msg(new std_msgs::String);

  while (ros::ok())
  {
    key = cvWaitKey(10);

    if (key == ' ')
    {
      std::cout << "Sending \"Space\" to ptam" << std::endl;
      msg->data = "Space";
      key_pub.publish(msg);
    }
    else if (key == 'r')
    {
      std::cout << "Sending \"r\" to ptam" << std::endl;
      msg->data = "r";
      key_pub.publish(msg);
    }
    else if (key == 'a')
    {
      std::cout << "Sending \"a\" to ptam" << std::endl;
      msg->data = "a";
      key_pub.publish(msg);
    }
    else if (key == 'q')
    {
      std::cout << "Sending \"q\" to ptam" << std::endl;
      msg->data = "q";
      key_pub.publish(msg);
    }
    else if (key == 's')
    {
      if (subscribed)
      {
        remote.unsubscribe();
        subscribed = false;
        std::cout << "unsubscribed" << std::endl;
      }
      else
      {
        remote.subscribe(n);
        subscribed = true;
        std::cout << "subscribed" << std::endl;
      }
    }

    ros::spinOnce();
  }
  return 0;
}
