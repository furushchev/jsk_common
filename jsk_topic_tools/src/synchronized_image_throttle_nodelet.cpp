/*
 * synchronized_image_throttle_nodelet.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_topic_tools/synchronized_image_throttle.h>


namespace jsk_topic_tools
{
  void SynchronizedImageThrottle::onInit()
  {
    DiagnosticNodelet::onInit();
    latest_stamp_ = ros::Time::now();
    onInitPostProcess();
  }

  void SynchronizedImageThrottle::subscribe()
  {
    sub_img1_.subscribe(*pnh_, "input/image1", 1);
    sub_img2_.subscribe(*pnh_, "input/image2", 1);

    if (use_camera_info_) {
      sub_info1_.subscribe(*pnh_, "input/camera_info1", 1);
      sub_info2_.subscribe(*pnh_, "input/camera_info2", 1);
    }
    if (approximate_sync_) {
      if (use_camera_info_) {
        async_camera_ =
          boost::make_shared<message_filters::Synchronizer<AsyncPolicyCamera> >(queue_size_);
        async_camera_->connectInput(sub_img1_, sub_info1_, sub_img2_, sub_info2_);
        async_camera_->registerCallback(boost::bind(&SynchronizedImageThrottle::synchronizeCamera,
                                                    this, _1, _2, _3, _4));
      } else {
        async_image_ =
          boost::make_shared<message_filters::Synchronizer<AsyncPolicyImage> >(queue_size_);
        async_image_->connectInput(sub_img1_, sub_img2_);
        async_image_->registerCallback(boost::bind(&SynchronizedImageThrottle::synchronizeImage,
                                                   this, _1, _2));
      }
    } else {
      if (use_camera_info_) {
        sync_camera_ =
          boost::make_shared<message_filters::Synchronizer<SyncPolicyCamera> >(queue_size_);
        sync_camera_->connectInput(sub_img1_, sub_info1_, sub_img2_, sub_info2_);
        sync_camera_->registerCallback(boost::bind(&SynchronizedImageThrottle::synchronizeCamera,
                                                   this, _1, _2, _3, _4));
      } else {
        sync_image_ =
          boost::make_shared<message_filters::Synchronizer<SyncPolicyImage> >(queue_size_);
        sync_image_->connectInput(sub_img1_, sub_img2_);
        sync_image_->registerCallback(boost::bind(&SynchronizedImageThrottle::synchronizeImage,
                                                  this, _1, _2));
      }
    }
  }

  void SynchronizedImageThrottle::unsubscribe()
  {
    sub_img1_.unsubscribe();
    sub_img2_.unsubscribe();
    if (use_camera_info_) {
      sub_info1_.unsubscribe();
      sub_info2_.unsubscribe();
    }
  }

  void SynchronizedImageThrottle::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool need_resubscribe =
      (queue_size_ != config.queue_size ||
       approximate_sync_ != config.approximate_sync ||
       use_camera_info_ != config.use_camera_info);

    queue_size_ = config.queue_size;
    approximate_sync_ = config.approximate_sync;
    use_camera_info_ = config.use_camera_info;
    update_rate_ = config.update_rate;
    force_synchronize_ = config.force_synchronize;

    if (isSubscribed() && need_resubscribe) {
      unsubscribe();
      subscribe();
    }
  }

  void SynchronizedImageThrottle::synchronizeImage(
      const sensor_msgs::Image::ConstPtr& img1,
      const sensor_msgs::Image::ConstPtr& img2)
  {
    // delegate lock to synchronizeCamera

    sensor_msgs::CameraInfo::Ptr null_info1;
    sensor_msgs::CameraInfo::Ptr null_info2;

    synchronizeCamera(img1, null_info1, img2, null_info2);
  }

  void SynchronizedImageThrottle::synchronizeCamera(
      const sensor_msgs::Image::ConstPtr& img1,
      const sensor_msgs::CameraInfo::ConstPtr& info1,
      const sensor_msgs::Image::ConstPtr& img2,
      const sensor_msgs::CameraInfo::ConstPtr& info2)
  {
    boost::mutex::scoped_lock lock(mutex_);

    ros::Time now = ros::Time::now();

    if (latest_stamp_ > now) {
      ROS_WARN("Detected jump back in time. latest_stamp_ is overwritten.");
      latest_stamp_ = now;
    }

    if (update_rate_ > 0.0 && (now - latest_stamp_).toSec() > 1.0 / update_rate_) {
      if (force_synchronize_) {
        sensor_msgs::Image img2_(*img2);
        img2_.header = img1->header;
        pub_img1_.publish(img1);
        pub_img2_.publish(img2);
        if (use_camera_info_) {
          sensor_msgs::CameraInfo info1_(*info1);
          sensor_msgs::CameraInfo info2_(*info2);
          info1_.header = img1->header;
          info2_.header = img1->header;
          pub_info1_.publish(info1);
          pub_info2_.publish(info2);
        }
      } else {
        pub_img1_.publish(img1);
        pub_img2_.publish(img2);
        if (use_camera_info_) {
          pub_info1_.publish(info1);
          pub_info2_.publish(info2);
        }
      }
      latest_stamp_ = now;
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_topic_tools::SynchronizedImageThrottle, nodelet::Nodelet);
