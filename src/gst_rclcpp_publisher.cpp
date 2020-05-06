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
 * Free Software Foundation, Inc., 51 Franklin Street, Suite 500,
 * Boston, MA 02110-1335, USA.
 */
/**
 * SECTION:element-gstrclcpp_publisher
 *
 * The rclcpp_publisher element does FIXME stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v fakesrc ! rclcpp_publisher ! FIXME ! fakesink
 * ]|
 * FIXME Describe what the pipeline does.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdexcept>

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideosink.h>

#include "rclcpp_gstreamer/image_encoding_conversions.hpp"
#include "rclcpp_gstreamer/gst_rclcpp_publisher.h"
#include "sensor_msgs/image_encodings.hpp"

GST_DEBUG_CATEGORY_STATIC (gst_rclcpp_publisher_debug_category);
#define GST_CAT_DEFAULT gst_rclcpp_publisher_debug_category

/* prototypes */


static void gst_rclcpp_publisher_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_rclcpp_publisher_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_rclcpp_publisher_dispose (GObject * object);
static void gst_rclcpp_publisher_finalize (GObject * object);
static void gst_rclcpp_publisher_set_node_name(GstRclcppPublisher* rclcpp_publisher,
                                               const GValue* value);
static void gst_rclcpp_publisher_set_topic_name(GstRclcppPublisher* rclcpp_publisher,
                                                const GValue* value);
static void gst_rclcpp_publisher_set_frame_id(GstRclcppPublisher* rclcpp_publisher,
                                              const GValue* value);

static GstFlowReturn gst_rclcpp_publisher_show_frame (GstVideoSink * video_sink,
    GstBuffer * buf);

enum
{
  PROP_UNDEFINED,
  PROP_NODE_NAME,
  PROP_TOPIC_NAME,
  PROP_IMAGE_FRAME_ID,
};

/* pad templates */

/* FIXME: add/remove formats you can handle */
// #define VIDEO_SINK_CAPS


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstRclcppPublisher, gst_rclcpp_publisher, GST_TYPE_VIDEO_SINK,
  GST_DEBUG_CATEGORY_INIT (gst_rclcpp_publisher_debug_category, "rclcpp_publisher", 0,
  "debug category for rclcpp_publisher element"));

static void
gst_rclcpp_publisher_class_init (GstRclcppPublisherClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstVideoSinkClass *video_sink_class = GST_VIDEO_SINK_CLASS (klass);

  std::stringstream ss;
  ss << "video/x-raw,width = (int)[1, 32768], height = (int)[1, 32768], framerate = (fraction)[ 0/1, 2147483647/1 ], channels = (int)[3, 4], format={";
  // ss << "YUY2, UYVY, RGBA, RGB, BGR, GRAY8, GRAY16_BE, GRAY16_LE" << "}";
  auto supported_encodings = EncodingConversions::supported_gst_encodings();
  for (size_t i = 0; i < supported_encodings.size(); ++i) {
    ss << supported_encodings[i];
    if (i < supported_encodings.size() - 1) {
      ss << ", ";
    }
  }
  ss << "}";
  // std::cout << "Creating caps with template: " << ss.str() << std::endl;
  /* Setting up pads and setting metadata should be moved to
     base_class_init if you intend to subclass this class. */
  auto caps = gst_caps_from_string(ss.str().c_str());
  gst_element_class_add_pad_template (GST_ELEMENT_CLASS(klass),
      gst_pad_template_new ("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS(klass),
      "rclcpp publisher", "Generic", "Publishes images to a ROS 2 topic from a gstreamer pipeline",
      "<brawner@robottimo.com>");

  gobject_class->set_property = gst_rclcpp_publisher_set_property;
  gobject_class->get_property = gst_rclcpp_publisher_get_property;
  gobject_class->dispose = gst_rclcpp_publisher_dispose;
  gobject_class->finalize = gst_rclcpp_publisher_finalize;
  video_sink_class->show_frame = gst_rclcpp_publisher_show_frame;

  g_object_class_install_property (gobject_class, PROP_NODE_NAME,
    g_param_spec_string ("node-name", "Node Name",
              "Name of the rclcpp publisher node (default = gst_publisher)",
              "gst_publisher", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_TOPIC_NAME,
    g_param_spec_string ("topic-name", "Topic Name",
              "Name of rclcpp topic (default = image)",
              "image", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property (gobject_class, PROP_IMAGE_FRAME_ID,
    g_param_spec_string ("image-frame-id", "Image Frame ID",
              "Name of frame id to set for image (default = gst-image)",
              "gst-image", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}

static void
gst_rclcpp_publisher_init (GstRclcppPublisher *rclcpp_publisher)
{
  std::cout << "Initializing rclcpp node" << std::endl;
  rclcpp::init(0, nullptr);
  rclcpp_publisher->node_name = "gst_publisher";
  rclcpp_publisher->topic_name = "image";
  rclcpp_publisher->image_frame_id = "gst-image";

  // Setup and initialize default node
  rclcpp_publisher->node = std::make_unique<GstPublisherNode>(
    rclcpp_publisher->node_name);
  rclcpp_publisher->node->set_topic_name(rclcpp_publisher->topic_name);
  rclcpp_publisher->node->set_frame_id(rclcpp_publisher->image_frame_id);
}

void gst_rclcpp_publisher_set_node_name(GstRclcppPublisher* rclcpp_publisher, const GValue *value) {
  g_assert(value);
  auto node_name_str = g_value_get_string(value);
  g_assert(node_name_str);
  g_assert(node_name_str[0] != '\0');
  g_assert(rclcpp_publisher->node);
  rclcpp_publisher->node_name = node_name_str;

  // Destroying old node, creating new one with new name
  rclcpp_publisher->node =
    std::make_unique<GstPublisherNode>(rclcpp_publisher->node_name);
  rclcpp_publisher->node->set_topic_name(rclcpp_publisher->topic_name);
  rclcpp_publisher->node->set_frame_id(rclcpp_publisher->image_frame_id);
}

void gst_rclcpp_publisher_set_topic_name(GstRclcppPublisher* rclcpp_publisher, const GValue * value) {
  g_assert(value);
  auto topic_name_str = g_value_get_string(value);
  g_assert(topic_name_str);
  g_assert(topic_name_str[0] != '\0');
  g_assert(rclcpp_publisher->node);
  rclcpp_publisher->node->set_topic_name(topic_name_str);
}

void gst_rclcpp_publisher_set_frame_id(GstRclcppPublisher* rclcpp_publisher, const GValue * value) {
  g_assert(value);
  auto frame_id_str = g_value_get_string(value);
  g_assert(frame_id_str);
  g_assert(frame_id_str[0] != '\0');
  rclcpp_publisher->image_frame_id = frame_id_str;
  rclcpp_publisher->node->base_image_.header.frame_id = rclcpp_publisher->image_frame_id;
}

void
gst_rclcpp_publisher_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (object);
  g_assert(value);
  switch (property_id) {
    case PROP_NODE_NAME:
      gst_rclcpp_publisher_set_node_name(rclcpp_publisher, value);
      break;
    case PROP_TOPIC_NAME:
      gst_rclcpp_publisher_set_topic_name(rclcpp_publisher, value);
      break;
    case PROP_IMAGE_FRAME_ID:
      gst_rclcpp_publisher_set_frame_id(rclcpp_publisher, value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rclcpp_publisher_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (object);

  GST_DEBUG_OBJECT (rclcpp_publisher, "get_property");

  switch (property_id) {
    case PROP_NODE_NAME:
      g_value_set_string(value, rclcpp_publisher->node_name.c_str());
      break;
    case PROP_TOPIC_NAME:
      g_value_set_string(value, rclcpp_publisher->topic_name.c_str());
      break;
    case PROP_IMAGE_FRAME_ID:
      g_value_set_string(value, rclcpp_publisher->image_frame_id.c_str());
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rclcpp_publisher_dispose (GObject * object)
{
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (object);

  GST_DEBUG_OBJECT (rclcpp_publisher, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_rclcpp_publisher_parent_class)->dispose (object);
}

static gboolean print_field (GQuark field, const GValue * value, gpointer pfx) {
  gchar *str = gst_value_serialize (value);

  g_print ("%s  %15s: %s\n", (gchar *) pfx, g_quark_to_string (field), str);
  g_free (str);
  return TRUE;
}

static void print_caps (const GstCaps * caps, const gchar * pfx) {
  guint i;

  g_return_if_fail (caps != NULL);

  if (gst_caps_is_any (caps)) {
    g_print ("%sANY\n", pfx);
    return;
  }
  if (gst_caps_is_empty (caps)) {
    g_print ("%sEMPTY\n", pfx);
    return;
  }

  g_print("caps size %d", gst_caps_get_size (caps));
  for (i = 0; i < gst_caps_get_size (caps); i++) {
    GstStructure *structure = gst_caps_get_structure (caps, i);
    g_print("%d", i);
    g_print ("%d %s%s\n", i, pfx, gst_structure_get_name (structure));
    gst_structure_foreach (structure, print_field, (gpointer) pfx);
  }
}

void
gst_rclcpp_publisher_finalize (GObject * object)
{
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (object);

  GST_DEBUG_OBJECT (rclcpp_publisher, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_rclcpp_publisher_parent_class)->finalize (object);
}

static GstFlowReturn
gst_rclcpp_publisher_show_frame (GstVideoSink * sink, GstBuffer * buf)
{
  auto start = std::chrono::steady_clock::now();
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (sink);
  GstClock* gst_clock = gst_element_get_clock(&sink->element.element);
  GstClockTime gst_nanoseconds = gst_clock_get_time(gst_clock);
  int diff = static_cast<int>(gst_nanoseconds) - static_cast<int>(rclcpp_publisher->prev_time_);
  rclcpp_publisher->prev_time_ = gst_nanoseconds;
  // std::cout << "Diff: " << diff /1000 << std::endl;
  if (rclcpp_publisher->gst_sync_time == 0) {
    rclcpp_publisher->gst_sync_time = gst_nanoseconds;
    rclcpp_publisher->rclcpp_sync_time = rclcpp_publisher->node->now();
  }

  const GstCaps * caps  = gst_pad_get_current_caps(sink->element.sinkpad);
  GstStructure * structure = gst_caps_get_structure (caps, 0);

  const char* format = gst_structure_get_string(structure, "format");
  if (!format) {
    g_print("No format available\n");
    print_caps(caps, "    ");
    return GST_FLOW_ERROR;
  }

  gint height = 0;
  gint width = 0;

  if (!gst_structure_get_int (structure, "width", &width) ||
      !gst_structure_get_int (structure, "height", &height)) {
    g_print ("No width/height available\n");
    print_caps(caps, "    ");
    return GST_FLOW_ERROR;
  }

  size_t gst_time_diff_ns = gst_nanoseconds - rclcpp_publisher->gst_sync_time;
  auto rclcpp_now = rclcpp_publisher->rclcpp_sync_time + rclcpp::Duration(gst_time_diff_ns);
  GstPublisherNode::Publish(rclcpp_publisher->node.get(), buf, width, height, format, rclcpp_now);
  auto end = std::chrono::steady_clock::now();
  // std::cout << "Show frame: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
  //           << "us" << std::endl;
  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  return gst_element_register (plugin, "rclcpp_publisher", GST_RANK_NONE,
      GST_TYPE_RCLCPP_PUBLISHER);
}

/* FIXME: these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
#ifndef VERSION
#define VERSION "0.1"
#endif
#ifndef PACKAGE
#define PACKAGE "rclcpp_publisher"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "rclcpp_publisher"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "http://FIXME.org/"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    rclcpp_publisher,
    "FIXME plugin description",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)

GstPublisherNode::GstPublisherNode(const std::string& name)
: rclcpp::Node(name) {}

GstPublisherNode::~GstPublisherNode() {}

void GstPublisherNode::set_topic_name(const std::string& topic_name) {
  publisher_ = image_transport::create_publisher(this, topic_name);
}

void GstPublisherNode::set_frame_id(const std::string& frame_id) {
  base_image_.header.frame_id = frame_id;
}

void GstPublisherNode::Publish(
    GstPublisherNode* node, GstBuffer* image_buffer, size_t width,
    size_t height, const char* format, rclcpp::Time rclcpp_time_stamp) {
  auto start = std::chrono::steady_clock::now();
  GstMapInfo info;
  g_assert(gst_buffer_map(image_buffer, &info, GST_MAP_READ));
  auto mem_map = std::chrono::steady_clock::now();

  node->base_image_.header.stamp = rclcpp_time_stamp;

  if (node->base_image_.height != height
      || node->base_image_.width != width
      || info.size != node->buffer_size_
      || strcmp(node->gst_format_.c_str(), format) != 0) {
    node->base_image_.height = height;
    node->base_image_.width = width;
    node->gst_format_ = format;
    node->buffer_size_ = info.size;
    node->base_image_.data.resize(info.size);
    node->base_image_.encoding = EncodingConversions::gst_to_ros(format);
    g_assert(!node->base_image_.encoding.empty());

    node->base_image_.step =
      width * sensor_msgs::image_encodings::numChannels(node->base_image_.encoding);
  }
  auto prep_image = std::chrono::steady_clock::now();

  auto resize = std::chrono::steady_clock::now();
  node->base_image_.data.assign(info.data, info.data + info.size);
  auto memcp = std::chrono::steady_clock::now();
  gst_buffer_unmap(image_buffer, &info);

  auto unref = std::chrono::steady_clock::now();
  node->publisher_.publish(node->base_image_);

  auto end = std::chrono::steady_clock::now();
  // std::cout << "Mem map: "
  //           <<  std::chrono::duration_cast<std::chrono::microseconds>(mem_map - start).count()
  //           << "us\n"
  //           << "Prep image: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(prep_image - mem_map).count()
  //           << "us\n"
  //           << "Memcpy: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(memcp - resize).count()
  //           << "us\n"
  //           << "Unref: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(unref - memcp).count()
  //           << "us\n"
  //           << "Publish: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end - unref).count()
  //           << "us" << std::endl;
}
