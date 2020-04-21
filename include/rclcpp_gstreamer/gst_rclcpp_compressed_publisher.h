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

#ifndef _GST_RCLCPP_COMPRESSED_PUBLISHER_H_
#define _GST_RCLCPP_COMPRESSED_PUBLISHER_H_

#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

G_BEGIN_DECLS

#define GST_TYPE_RCLCPP_COMPRESSED_PUBLISHER   (gst_RCLCPP_COMPRESSED_PUBLISHER_get_type())
#define GST_RCLCPP_COMPRESSED_PUBLISHER(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_RCLCPP_COMPRESSED_PUBLISHER,GstRclcppCompressedPublisher))
#define GST_RCLCPP_COMPRESSED_PUBLISHER_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_RCLCPP_COMPRESSED_PUBLISHER,GstRclcppCompressedPublisherClass))
#define GST_IS_RCLCPP_COMPRESSED_PUBLISHER(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_RCLCPP_COMPRESSED_PUBLISHER))
#define GST_IS_RCLCPP_COMPRESSED_PUBLISHER_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_RCLCPP_COMPRESSED_PUBLISHER))


class GstCompressedPublisherNode : public rclcpp::Node {
public:
  GstCompressedPublisherNode(const std::string& name, const std::string& topic_name);
  ~GstCompressedPublisherNode();

  static void Publish(GstCompressedPublisherNode* node, GstBuffer* image_buffer, size_t width, size_t height);

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> publisher_;
};

typedef struct _GstRclcppCompressedPublisher GstRclcppCompressedPublisher;
typedef struct _GstRclcppCompressedPublisherClass GstRclcppCompressedPublisherClass;

struct _GstRclcppCompressedPublisher
{
  GstVideoSink base_rclcpppublisher;
  std::unique_ptr<GstCompressedPublisherNode> node;
};

struct _GstRclcppCompressedPublisherClass
{
  GstVideoSinkClass base_rclcpppublisher_class;
};

GType gst_RCLCPP_COMPRESSED_PUBLISHER_get_type (void);

G_END_DECLS


#endif
