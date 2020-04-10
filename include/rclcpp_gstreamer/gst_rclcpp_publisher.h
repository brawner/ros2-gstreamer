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

#ifndef _GST_RCLCPP_PUBLISHER_H_
#define _GST_RCLCPP_PUBLISHER_H_

#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

G_BEGIN_DECLS

#define GST_TYPE_RCLCPP_PUBLISHER   (gst_rclcpp_publisher_get_type())
#define GST_RCLCPP_PUBLISHER(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_RCLCPP_PUBLISHER,GstRclcppPublisher))
#define GST_RCLCPP_PUBLISHER_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_RCLCPP_PUBLISHER,GstRclcppPublisherClass))
#define GST_IS_RCLCPP_PUBLISHER(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_RCLCPP_PUBLISHER))
#define GST_IS_RCLCPP_PUBLISHER_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_RCLCPP_PUBLISHER))


class GstPublisherNode : public rclcpp::Node {
public:
  GstPublisherNode(const std::string& name, const std::string& topic_name);
  ~GstPublisherNode();

  static void Publish(GstPublisherNode* node, GstBuffer* image_buffer, size_t width, size_t height);

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher_;
};

typedef struct _GstRclcppPublisher GstRclcppPublisher;
typedef struct _GstRclcppPublisherClass GstRclcppPublisherClass;

struct _GstRclcppPublisher
{
  GstVideoSink base_rclcpppublisher;
  std::unique_ptr<GstPublisherNode> node;
};

struct _GstRclcppPublisherClass
{
  GstVideoSinkClass base_rclcpppublisher_class;
};

GType gst_rclcpp_publisher_get_type (void);

G_END_DECLS


#endif
