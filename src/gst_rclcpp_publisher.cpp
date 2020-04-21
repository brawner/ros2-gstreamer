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

static GstFlowReturn gst_rclcpp_publisher_show_frame (GstVideoSink * video_sink,
    GstBuffer * buf);

enum
{
  PROP_0
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
  std::cout << "Creating caps with template: " << ss.str() << std::endl;
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
  video_sink_class->show_frame = GST_DEBUG_FUNCPTR (gst_rclcpp_publisher_show_frame);
}

static void
gst_rclcpp_publisher_init (GstRclcppPublisher *rclcpp_publisher)
{
  rclcpp::init(0, nullptr);
  rclcpp_publisher->node = std::make_unique<GstPublisherNode>("gst_publisher", "image");
}

void
gst_rclcpp_publisher_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (object);

  GST_DEBUG_OBJECT (rclcpp_publisher, "set_property");

  switch (property_id) {
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
  GstRclcppPublisher *rclcpp_publisher = GST_RCLCPP_PUBLISHER (sink);

  GST_DEBUG_OBJECT (rclcpp_publisher, "show_frame");
  GstPad * pad = gst_element_get_static_pad (reinterpret_cast<GstElement*>(sink), "sink");
  g_assert(pad);
  const GstCaps * caps  = gst_pad_get_current_caps(pad);
  GstStructure * structure = gst_caps_get_structure (caps, 0);
  gint height = 0;
  gint width = 0;
  const char* format = gst_structure_get_string(structure, "format");
  if (!format) {
    g_print("No format available\n");
    print_caps(caps, "    ");
    return GST_FLOW_ERROR;
  }

  if (!gst_structure_get_int (structure, "width", &width) ||
      !gst_structure_get_int (structure, "height", &height)) {
    g_print ("No width/height available\n");
    print_caps(caps, "    ");
    return GST_FLOW_ERROR;
  }
  // g_print("Caps dimensions %d x %d", width, height);
  GstPublisherNode::Publish(rclcpp_publisher->node.get(), buf, width, height, format);

  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
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

GstPublisherNode::GstPublisherNode(const std::string& name, const std::string& topic_name)
: rclcpp::Node(name), publisher_(nullptr) {
  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name, 1);
}

GstPublisherNode::~GstPublisherNode() {}

void GstPublisherNode::Publish(
    GstPublisherNode* node, GstBuffer* image_buffer, size_t width, size_t height, const char* format) {
  GstMemory *memory = gst_buffer_get_memory(image_buffer, 0);
  GstMapInfo info;
  gst_memory_map(memory, &info, GST_MAP_READ);
  gsize &buffer_size = info.size;
  guint8 *&buffer_data = info.data;

  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera";
  image.height = height;
  image.width = width;

  auto encoding = EncodingConversions::gst_to_ros(format);
  if (encoding.empty()) {
    throw std::runtime_error("Encoding conversion failed");
  }
  image.encoding = encoding;
  auto num_channels = sensor_msgs::image_encodings::numChannels(encoding);
  auto depth = sensor_msgs::image_encodings::bitDepth(encoding) / 8;
  image.step = width * num_channels;
  image.data.resize(buffer_size);

  // g_print("Buffer size %d",  gst_buffer_get_size(image_buffer));
  size_t size = gst_buffer_get_size(image_buffer);
  memcpy(&image.data[0], buffer_data, buffer_size);
  // g_print("Publishing image\n");
  node->publisher_->publish(image);
  // Clean up
  gst_memory_unmap(memory, &info);
  gst_memory_unref(memory);
}
