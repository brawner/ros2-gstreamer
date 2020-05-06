/* GStreamer
 * Copyright (C) 2020 FIXME <fixme@example.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef _GST_RCLCPP_SUBSCRIBER_H_
#define _GST_RCLCPP_SUBSCRIBER_H_

#include <gst/base/gstpushsrc.h>

#include <memory>
#include <queue>
#include <variant>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

G_BEGIN_DECLS

#define GST_TYPE_RCLCPP_SUBSCRIBER   (gst_rclcpp_subscriber_get_type())
#define GST_RCLCPP_SUBSCRIBER(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_RCLCPP_SUBSCRIBER,GstRclcppSubscriber))
#define GST_RCLCPP_SUBSCRIBER_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_RCLCPP_SUBSCRIBER,GstRclcppSubscriberClass))
#define GST_IS_RCLCPP_SUBSCRIBER(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_RCLCPP_SUBSCRIBER))
#define GST_IS_RCLCPP_SUBSCRIBER_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_RCLCPP_SUBSCRIBER))


enum class CapsState: int {
  UNITIALIZED = 0,
  DUMMY_CAPS = 1,
  INITIALIZED = 2,
};

using SupportedImageType =
  std::variant<sensor_msgs::msg::Image::ConstSharedPtr,
               sensor_msgs::msg::CompressedImage::ConstSharedPtr>;

class GstSubscriberNode : public rclcpp::Node {
public:
  GstSubscriberNode(const std::string& name,
                    std::shared_ptr<std::queue<SupportedImageType>> queue);
  ~GstSubscriberNode();

  void set_image_transport(const std::string& topic_name, const std::string& transport_type = "raw");
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void on_compressed_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
private:
  void SetBufferSizeTo(size_t max_size);
  void RecordImageTime(rclcpp::Time time_point);
  image_transport::Subscriber subscriber_;
  std::shared_ptr<std::queue<SupportedImageType>> queue_;
  rclcpp::Time prev_time_;
};

typedef struct _GstRclcppSubscriber GstRclcppSubscriber;
typedef struct _GstRclcppSubscriberClass GstRclcppSubscriberClass;

struct _GstRclcppSubscriber
{
  GstPushSrc base_rclcppsubscriber;
  std::shared_ptr<std::queue<SupportedImageType>> queue;
  std::shared_ptr<GstSubscriberNode> node;
  CapsState initialized_caps;

  std::string node_name;
  std::string topic_name;
  std::string transport_type;
};

struct _GstRclcppSubscriberClass
{
  GstPushSrcClass base_rclcppsubscriber_class;
};

GType gst_rclcpp_subscriber_get_type (void);

G_END_DECLS

#endif
