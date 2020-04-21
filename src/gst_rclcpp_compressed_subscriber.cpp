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
 * SECTION:element-gstrclcpp_subscriber
 *
 * The rclcpp_subscriber element does FIXME stuff.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch-1.0 -v fakesrc ! rclcpp_subscriber ! FIXME ! fakesink
 * ]|
 * FIXME Describe what the pipeline does.
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include "rclcpp_gstreamer/gst_rclcpp_compressed_subscriber.h"
#include "rclcpp_gstreamer/image_encoding_conversions.hpp"

GST_DEBUG_CATEGORY_STATIC (gst_rclcpp_subscriber_debug_category);
#define GST_CAT_DEFAULT gst_rclcpp_subscriber_debug_category

/* prototypes */


static void gst_rclcpp_subscriber_set_property (GObject * object,
    guint property_id, const GValue * value, GParamSpec * pspec);
static void gst_rclcpp_subscriber_get_property (GObject * object,
    guint property_id, GValue * value, GParamSpec * pspec);
static void gst_rclcpp_subscriber_dispose (GObject * object);
static void gst_rclcpp_subscriber_finalize (GObject * object);

// static GstCaps *gst_rclcpp_subscriber_get_caps (GstPushSrc * src, GstCaps * filter);
// static gboolean gst_rclcpp_subscriber_negotiate (GstPushSrc * src);
// static GstCaps *gst_rclcpp_subscriber_fixate (GstPushSrc * src, GstCaps * caps);
// static gboolean gst_rclcpp_subscriber_set_caps (GstPushSrc * src, GstCaps * caps);
// static gboolean gst_rclcpp_subscriber_decide_allocation (GstPushSrc * src,
//     GstQuery * query);
// static gboolean gst_rclcpp_subscriber_start (GstPushSrc * src);
// static gboolean gst_rclcpp_subscriber_stop (GstPushSrc * src);
// static void gst_rclcpp_subscriber_get_times (GstPushSrc * src, GstBuffer * buffer,
//     GstClockTime * start, GstClockTime * end);
// static gboolean gst_rclcpp_subscriber_get_size (GstPushSrc * src, guint64 * size);
// static gboolean gst_rclcpp_subscriber_is_seekable (GstPushSrc * src);
// static gboolean gst_rclcpp_subscriber_prepare_seek_segment (GstPushSrc * src,
//     GstEvent * seek, GstSegment * segment);
// static gboolean gst_rclcpp_subscriber_do_seek (GstPushSrc * src, GstSegment * segment);
// static gboolean gst_rclcpp_subscriber_unlock (GstPushSrc * src);
// static gboolean gst_rclcpp_subscriber_unlock_stop (GstPushSrc * src);
// static gboolean gst_rclcpp_subscriber_query (GstPushSrc * src, GstQuery * query);
// static gboolean gst_rclcpp_subscriber_event (GstPushSrc * src, GstEvent * event);
// static GstFlowReturn gst_rclcpp_subscriber_create (GstPushSrc * src, guint64 offset,
//     guint size, GstBuffer ** buf);
// static GstFlowReturn gst_rclcpp_subscriber_alloc (GstPushSrc * src, guint64 offset,
//     guint size, GstBuffer ** buf);
static GstFlowReturn gst_rclcpp_subscriber_fill (GstPushSrc * src, GstBuffer * buf);

enum
{
  PROP_0
};

std::stringstream ss;
ss << "video/x-raw,width = (int)[1, 32768], height = (int)[1, 32768], framerate = (fraction)[ 0/1, 2147483647/1 ], channels = (int)[3, 4], format={";
for (auto encoding : EncodingConversions::supported_gst_compressed_encodings()) {
  ss << encoding << ", ";
}
ss << "}"

/* pad templates */
auto caps = gst_caps_from_string(ss.string.c_str());
static GstStaticPadTemplate gst_rclcpp_subscriber_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    caps
    );


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstRclcppSubscriber, gst_rclcpp_subscriber, GST_TYPE_PUSH_SRC,
  GST_DEBUG_CATEGORY_INIT (gst_rclcpp_subscriber_debug_category, "rclcpp_subscriber", 0,
  "debug category for rclcpp_subscriber element"));

static void
gst_rclcpp_subscriber_class_init (GstRclcppSubscriberClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstPushSrcClass *push_src_class = GST_PUSH_SRC_CLASS (klass);

  /* Setting up pads and setting metadata should be moved to
     push_class_init if you intend to subclass this class. */
  gst_element_class_add_static_pad_template (GST_ELEMENT_CLASS(klass),
      &gst_rclcpp_subscriber_src_template);

  gst_element_class_set_static_metadata (GST_ELEMENT_CLASS(klass),
      "FIXME Long name", "Generic", "FIXME Description",
      "FIXME <fixme@example.com>");

  gobject_class->set_property = gst_rclcpp_subscriber_set_property;
  gobject_class->get_property = gst_rclcpp_subscriber_get_property;
  gobject_class->dispose = gst_rclcpp_subscriber_dispose;
  gobject_class->finalize = gst_rclcpp_subscriber_finalize;
  // push_src_class->get_caps = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_get_caps);
  // push_src_class->negotiate = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_negotiate);
  // push_src_class->fixate = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_fixate);
  // push_src_class->set_caps = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_set_caps);
  // push_src_class->decide_allocation = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_decide_allocation);
  // push_src_class->start = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_start);
  // push_src_class->stop = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_stop);
  // push_src_class->get_times = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_get_times);
  // push_src_class->get_size = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_get_size);
  // push_src_class->is_seekable = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_is_seekable);
  // push_src_class->prepare_seek_segment = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_prepare_seek_segment);
  // push_src_class->do_seek = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_do_seek);
  // push_src_class->unlock = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_unlock);
  // push_src_class->unlock_stop = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_unlock_stop);
  // push_src_class->query = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_query);
  // push_src_class->event = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_event);
  // push_src_class->create = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_create);
  // push_src_class->alloc = GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_alloc);


  push_src_class->fill =  GST_DEBUG_FUNCPTR (gst_rclcpp_subscriber_fill);

}

static void
gst_rclcpp_subscriber_init (GstRclcppSubscriber *rclcpp_subscriber)
{
  rclcpp::init(0, nullptr);
  rclcpp_subscriber->queue = std::make_shared<std::queue<sensor_msgs::msg::CompressedImage::SharedPtr>>();
  rclcpp_subscriber->node =
    std::make_shared<GstCompressedSubscriberNode>("gst_subscriber", "image", rclcpp_subscriber->queue);
}

void
gst_rclcpp_subscriber_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (object);

  GST_DEBUG_OBJECT (rclcpp_subscriber, "set_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rclcpp_subscriber_get_property (GObject * object, guint property_id,
    GValue * value, GParamSpec * pspec)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (object);

  GST_DEBUG_OBJECT (rclcpp_subscriber, "get_property");

  switch (property_id) {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, property_id, pspec);
      break;
  }
}

void
gst_rclcpp_subscriber_dispose (GObject * object)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (object);

  GST_DEBUG_OBJECT (rclcpp_subscriber, "dispose");

  /* clean up as possible.  may be called multiple times */

  G_OBJECT_CLASS (gst_rclcpp_subscriber_parent_class)->dispose (object);
}

void
gst_rclcpp_subscriber_finalize (GObject * object)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (object);

  GST_DEBUG_OBJECT (rclcpp_subscriber, "finalize");

  /* clean up object here */

  G_OBJECT_CLASS (gst_rclcpp_subscriber_parent_class)->finalize (object);
}
//
// /* get caps from subclass */
// static GstCaps *
// gst_rclcpp_subscriber_get_caps (GstPushSrc * src, GstCaps * filter)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "get_caps");
//
//   return NULL;
// }
//
// /* decide on caps */
// static gboolean
// gst_rclcpp_subscriber_negotiate (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "negotiate");
//
//   return TRUE;
// }
//
// /* called if, in negotiation, caps need fixating */
// static GstCaps *
// gst_rclcpp_subscriber_fixate (GstPushSrc * src, GstCaps * caps)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "fixate");
//
//   return NULL;
// }
//
// /* notify the subclass of new caps */
// static gboolean
// gst_rclcpp_subscriber_set_caps (GstPushSrc * src, GstCaps * caps)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "set_caps");
//
//   return TRUE;
// }
//
// /* setup allocation query */
// static gboolean
// gst_rclcpp_subscriber_decide_allocation (GstPushSrc * src, GstQuery * query)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "decide_allocation");
//
//   return TRUE;
// }
//
// /* start and stop processing, ideal for opening/closing the resource */
// static gboolean
// gst_rclcpp_subscriber_start (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "start");
//
//   return TRUE;
// }
//
// static gboolean
// gst_rclcpp_subscriber_stop (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "stop");
//
//   return TRUE;
// }
//
// /* given a buffer, return start and stop time when it should be pushed
//  * out. The push class will sync on the clock using these times. */
// static void
// gst_rclcpp_subscriber_get_times (GstPushSrc * src, GstBuffer * buffer,
//     GstClockTime * start, GstClockTime * end)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "get_times");
//
// }
//
// /* get the total size of the resource in bytes */
// static gboolean
// gst_rclcpp_subscriber_get_size (GstPushSrc * src, guint64 * size)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "get_size");
//
//   return TRUE;
// }
//
// /* check if the resource is seekable */
// static gboolean
// gst_rclcpp_subscriber_is_seekable (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "is_seekable");
//
//   return TRUE;
// }
//
// /* Prepare the segment on which to perform do_seek(), converting to the
//  * current pushsrc format. */
// static gboolean
// gst_rclcpp_subscriber_prepare_seek_segment (GstPushSrc * src, GstEvent * seek,
//     GstSegment * segment)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "prepare_seek_segment");
//
//   return TRUE;
// }
//
// /* notify subclasses of a seek */
// static gboolean
// gst_rclcpp_subscriber_do_seek (GstPushSrc * src, GstSegment * segment)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "do_seek");
//
//   return TRUE;
// }
//
// /* unlock any pending access to the resource. subclasses should unlock
//  * any function ASAP. */
// static gboolean
// gst_rclcpp_subscriber_unlock (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "unlock");
//
//   return TRUE;
// }
//
// /* Clear any pending unlock request, as we succeeded in unlocking */
// static gboolean
// gst_rclcpp_subscriber_unlock_stop (GstPushSrc * src)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "unlock_stop");
//
//   return TRUE;
// }
//
// /* notify subclasses of a query */
// static gboolean
// gst_rclcpp_subscriber_query (GstPushSrc * src, GstQuery * query)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "query");
//
//   return TRUE;
// }
//
// /* notify subclasses of an event */
// static gboolean
// gst_rclcpp_subscriber_event (GstPushSrc * src, GstEvent * event)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "event");
//
//   return TRUE;
// }
//
// /* ask the subclass to create a buffer with offset and size, the default
//  * implementation will call alloc and fill. */
// static GstFlowReturn
// gst_rclcpp_subscriber_create (GstPushSrc * src, guint64 offset, guint size,
//     GstBuffer ** buf)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "create");
//
//   return GST_FLOW_OK;
// }
//
// /* ask the subclass to allocate an output buffer. The default implementation
//  * will use the negotiated allocator. */
// static GstFlowReturn
// gst_rclcpp_subscriber_alloc (GstPushSrc * src, guint64 offset, guint size,
//     GstBuffer ** buf)
// {
//   GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
//
//   GST_DEBUG_OBJECT (rclcpp_subscriber, "alloc");
//
//   return GST_FLOW_OK;
// }

/* ask the subclass to fill the buffer with data from offset and size */
static GstFlowReturn
gst_rclcpp_subscriber_fill (GstPushSrc * src, GstBuffer * buf)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
  if (rclcpp_subscriber->queue->empty()) {
    rclcpp::spin_some(rclcpp_subscriber->node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return GST_FLOW_OK;
  }

  auto image = rclcpp_subscriber->queue->front();
  rclcpp_subscriber->queue->pop();

  GstMapInfo minfo;
  gst_buffer_map (buf, &minfo, GST_MAP_WRITE);
  // if (image->data.size() != minfo.size) {
  //   std::cout << image->data.size() << " != " << minfo.size << std::endl;
  //   g_assert(image->data.size() == minfo.size);
  // }
  memcpy(minfo.data, &image->data[0], image->date.size();
  //gst_buffer_fill(buf, 0, &image->data[0], (int) minfo.size);
  GST_DEBUG_OBJECT (rclcpp_subscriber, "fill");
  gst_buffer_unmap (buf, &minfo);

  return GST_FLOW_OK;
}

static gboolean
plugin_init (GstPlugin * plugin)
{

  /* FIXME Remember to set the rank if it's an element that is meant
     to be autoplugged by decodebin. */
  return gst_element_register (plugin, "rclcpp_subscriber", GST_RANK_NONE,
      GST_TYPE_RCLCPP_SUBSCRIBER);
}

/* FIXME: these are normally defined by the GStreamer build system.
   If you are creating an element to be included in gst-plugins-*,
   remove these, as they're always defined.  Otherwise, edit as
   appropriate for your external plugin package. */
#ifndef VERSION
#define VERSION "0.0.1"
#endif
#ifndef PACKAGE
#define PACKAGE "rclcpp_subscriber"
#endif
#ifndef PACKAGE_NAME
#define PACKAGE_NAME "rclcpp_subscriber"
#endif
#ifndef GST_PACKAGE_ORIGIN
#define GST_PACKAGE_ORIGIN "http://FIXME.org/"
#endif

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    rclcpp_subscriber,
    "FIXME plugin description",
    plugin_init, VERSION, "LGPL", PACKAGE_NAME, GST_PACKAGE_ORIGIN)

GstCompressedSubscriberNode::GstCompressedSubscriberNode(
  const std::string& name, const std::string& topic_name, std::shared_ptr<std::queue<sensor_msgs::msg::CompressedImage::SharedPtr>> queue)
: rclcpp::Node(name), subscriber_(nullptr), queue_(queue) {
  std::cout << "Subscribing to topic: " << topic_name << std::endl;
  subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(topic_name, 1, std::bind(&GstCompressedSubscriberNode::on_image, this, std::placeholders::_1));
}

GstCompressedSubscriberNode::~GstCompressedSubscriberNode() {}

void GstCompressedSubscriberNode::on_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  while(queue_->size() > 1) {
    queue_->pop();
  }
  queue_->push(msg);
}
