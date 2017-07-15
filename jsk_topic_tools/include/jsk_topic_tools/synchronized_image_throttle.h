/*
 * synchronized_image_throttle.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef SYNCHRONIZED_IMAGE_THROTTLE_H__
#define SYNCHRONIZED_IMAGE_THROTTLE_H__

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/SynchronizedImageThrottleConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace jsk_topic_tools
{
  class SynchronizedImageThrottle: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef SynchronizedImageThrottleConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image> SyncPolicyImage;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyCamera;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image> AsyncPolicyImage;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::CameraInfo,
      sensor_msgs::Image, sensor_msgs::CameraInfo> AsyncPolicyCamera;

    SynchronizedImageThrottle() : DiagnosticNodelet("SynchronizedImageThrottle"){}

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void synchronizeImage(
      const sensor_msgs::Image::ConstPtr& img1,
      const sensor_msgs::Image::ConstPtr& img2);
    virtual void synchronizeCamera(
      const sensor_msgs::Image::ConstPtr& img1,
      const sensor_msgs::CameraInfo::ConstPtr& info1,
      const sensor_msgs::Image::ConstPtr& img2,
      const sensor_msgs::CameraInfo::ConstPtr& info2);

    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    ros::Publisher pub_img1_, pub_img2_;
    ros::Publisher pub_info1_, pub_info2_;

    message_filters::Subscriber<sensor_msgs::Image> sub_img1_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img2_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info1_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info2_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyImage> > sync_image_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyCamera> > sync_camera_;
    boost::shared_ptr<message_filters::Synchronizer<AsyncPolicyImage> > async_image_;
    boost::shared_ptr<message_filters::Synchronizer<AsyncPolicyCamera> > async_camera_;

    ros::Time latest_stamp_;
    int queue_size_;
    bool approximate_sync_;
    bool use_camera_info_;
    bool force_synchronize_;
    double update_rate_;
  };
}

#endif // SYNCHRONIZED_IMAGE_THROTTLE_H__
