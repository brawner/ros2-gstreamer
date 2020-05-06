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

#include <sstream>

#include <gst/gst.h>
#include <gst/gstclock.h>
#include <gst/base/gstpushsrc.h>
#include "rclcpp_gstreamer/gst_rclcpp_subscriber.h"
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
static void rclcpp_subscriber_set_node_name(GstRclcppSubscriber *rclcpp_subscriber,
                                            const GValue* node_name_value);
static void rclcpp_subscriber_set_topic_name(GstRclcppSubscriber *rclcpp_subscriber,
                                             const GValue* topic_value);
// static GstCaps *gst_rclcpp_subscriber_get_caps (GstPushSrc * src, GstCaps * filter);
// static gboolean gst_rclcpp_subscriber_negotiate (GstPushSrc * src);
static GstCaps *gst_rclcpp_subscriber_fixate (GstBaseSrc * src, GstCaps * caps);
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
  PROP_UNDEFINED,
  PROP_NODE_NAME,
  PROP_TOPIC_NAME,
};

const std::string construct_caps_str() {
  std::stringstream ss;
   ss << "video/x-raw,width = (int)[1, 32768], height = (int)[1, 32768], framerate = (fraction)[ 0/1, 2147483647/1 ], channels = (int)[1, 4], format={";
   auto supported_encodings = EncodingConversions::supported_gst_encodings();
   for (size_t i = 0; i < supported_encodings.size(); ++i) {
     ss << supported_encodings[i];
     if (i < supported_encodings.size() - 1) {
       ss << ", ";
     }
   }
   ss << "}";
   std::cout << "Creating caps with template: " << ss.str() << std::endl;
  return ss.str();
}

/* pad templates */
auto caps = gst_caps_from_string(construct_caps_str().c_str());
static GstStaticPadTemplate gst_rclcpp_subscriber_src_template =
GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("video/x-raw"));


/* class initialization */

G_DEFINE_TYPE_WITH_CODE (GstRclcppSubscriber, gst_rclcpp_subscriber, GST_TYPE_PUSH_SRC,
  GST_DEBUG_CATEGORY_INIT (gst_rclcpp_subscriber_debug_category, "rclcpp_subscriber", 0,
  "debug category for rclcpp_subscriber element"))

static void
gst_rclcpp_subscriber_class_init (GstRclcppSubscriberClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseSrcClass *base_src_class = GST_BASE_SRC_CLASS(klass);
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
  base_src_class->fixate = gst_rclcpp_subscriber_fixate;
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

  g_object_class_install_property (gobject_class, PROP_NODE_NAME,
    g_param_spec_string ("node-name", "Node Name",
              "Name of the rclcpp publisher node (default = gst_publisher)",
              "gst_publisher", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property (gobject_class, PROP_TOPIC_NAME,
    g_param_spec_string ("topic-name", "Topic Name",
              "Name of rclcpp topic (default = image)",
              "image", static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

}

static void
gst_rclcpp_subscriber_init (GstRclcppSubscriber *rclcpp_subscriber)
{
  rclcpp::init(0, nullptr);
  rclcpp_subscriber->queue = std::make_shared<std::queue<sensor_msgs::msg::Image::ConstSharedPtr>>();
  rclcpp_subscriber->node = nullptr;
  rclcpp_subscriber->topic_name = "image";
  rclcpp_subscriber->initialized_caps = CapsState::UNITIALIZED;
  auto* base_push_src = &rclcpp_subscriber->base_rclcppsubscriber.parent;
  gst_pad_activate_mode (base_push_src->srcpad, GST_PAD_MODE_PUSH, TRUE);
  base_push_src->is_live = TRUE;
  base_push_src->random_access = FALSE;
  base_push_src->can_activate_push = TRUE;
  gst_pad_use_fixed_caps(base_push_src->srcpad);

  // Create node with default name and topic name
  rclcpp_subscriber->node =
    std::make_shared<GstSubscriberNode>("gst_publisher", rclcpp_subscriber->queue);
  rclcpp_subscriber->node->set_topic_name(rclcpp_subscriber->topic_name);
}

void rclcpp_subscriber_set_node_name(GstRclcppSubscriber *rclcpp_subscriber, const GValue* node_name_value) {
  g_assert(node_name_value);
  auto node_name = g_value_get_string(node_name_value);
  g_assert(node_name);
  g_assert(node_name[0] != '\0');

  std::cout << "Creating subscriber node with name: " << node_name << std::endl;
  rclcpp_subscriber->node =
    std::make_shared<GstSubscriberNode>(node_name, rclcpp_subscriber->queue);
  rclcpp_subscriber->node->set_topic_name(rclcpp_subscriber->topic_name);
}

void rclcpp_subscriber_set_topic_name(GstRclcppSubscriber *rclcpp_subscriber,
                                      const GValue* topic_value) {
  g_assert(topic_value);
  auto topic_str = g_value_get_string(topic_value);
  g_assert(topic_str);
  g_assert(topic_str[0] != '\0');
  g_assert(rclcpp_subscriber->node);
  std::cout << "Subscribing to topic name: " << topic_str << std::endl;
  rclcpp_subscriber->topic_name = topic_str;
  rclcpp_subscriber->node->set_topic_name(rclcpp_subscriber->topic_name);
}

void
gst_rclcpp_subscriber_set_property (GObject * object, guint property_id,
    const GValue * value, GParamSpec * pspec)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (object);

  GST_DEBUG_OBJECT (rclcpp_subscriber, "set_property");
  switch (property_id) {
    case PROP_NODE_NAME:
      rclcpp_subscriber_set_node_name(rclcpp_subscriber, value);
      break;
    case PROP_TOPIC_NAME:
      rclcpp_subscriber_set_topic_name(rclcpp_subscriber, value);
      break;
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
    case PROP_NODE_NAME:
      g_value_set_string(value, rclcpp_subscriber->node_name.c_str());
      break;
    case PROP_TOPIC_NAME:
      g_value_set_string(value, rclcpp_subscriber->topic_name.c_str());
      break;
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

static void
gst_rclcpp_subscriber_get_caps_info(const sensor_msgs::msg::Image& image,
    gint* width, gint* height, std::string* encoding, gint* channels) {
  *encoding = EncodingConversions::ros_to_gst(image.encoding).c_str();
  *width = image.width;
  *height = image.height;
  *channels = (image.step / image.width);
}

static void
gst_rclcpp_subscriber_set_caps(GstCaps* caps,
    size_t width, size_t height, const char* encoding, size_t channels) {
  // std::stringstream ss;
  // ss << "video/x-raw,framerate=30/1,format=" << encoding << ",width=" << width
  //    << ",height=" << height << "channels=" << channels;
  // GstStructure* structure = gst_structure_from_string(ss.str().c_str());
  // if (!structure) {
  //   std::cerr << "Failed to create a GstStructure from: " << ss.str() << std::endl;
  //   return;
  // }
  GstCaps* new_caps = gst_caps_new_simple ("video/x-raw",
     "format", G_TYPE_STRING, encoding,
     "framerate", GST_TYPE_FRACTION, 30, 1,
     "width", G_TYPE_INT, width,
     "height", G_TYPE_INT, height,
     "channels", G_TYPE_INT, channels,
     NULL);
  if (new_caps) {
    std::cout << "Updating new caps: " << std::endl;
    print_caps(new_caps, "    ");
    gst_caps_replace(&caps, new_caps);
    print_caps(caps, "    ");
  }
}

// /* called if, in negotiation, caps need fixating */
static GstCaps *
gst_rclcpp_subscriber_fixate (GstBaseSrc * src, GstCaps * caps)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
  std::cout << "Initialized?: " << static_cast<int>(rclcpp_subscriber->initialized_caps) << std::endl;
  GstCaps* new_caps = nullptr;
  if (!rclcpp_subscriber->queue->empty()) {
    auto image = rclcpp_subscriber->queue->front();

    gint width = 0;
    gint height = 0;
    gint channels = 0;
    std::string encoding;
    // GstStructure *structure = gst_caps_get_structure (caps, 0);
    gst_rclcpp_subscriber_get_caps_info(*image, &width, &height, &encoding, &channels);
    new_caps = gst_caps_new_simple ("video/x-raw",
       "format", G_TYPE_STRING, encoding.c_str(),
       "framerate", GST_TYPE_FRACTION, 30, 1,
       "width", G_TYPE_INT, width,
       "height", G_TYPE_INT, height,
       "channels", G_TYPE_INT, channels,
       NULL);
    rclcpp_subscriber->initialized_caps = CapsState::INITIALIZED;


    // gst_rclcpp_subscriber_set_caps(caps, width, height, encoding.c_str(), channels);
    // gst_structure_fixate_field_nearest_fraction(structure, "framerate", 30, 1);
    std::cout << "Setting caps: " << std::endl;
    g_assert(new_caps);
    print_caps(new_caps, "    ");
    GST_DEBUG_OBJECT (rclcpp_subscriber, "fixate");
    new_caps = GST_BASE_SRC_CLASS (gst_rclcpp_subscriber_parent_class)->fixate (src, new_caps);

    return new_caps;
  } else if (rclcpp_subscriber->queue->empty()
      && rclcpp_subscriber->initialized_caps != CapsState::INITIALIZED) {
    std::cout << "No image yet, setting dummy caps" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(rclcpp_subscriber->node);
    new_caps = gst_caps_new_simple ("video/x-raw",
       "format", G_TYPE_STRING, "RGBA",
       "framerate", GST_TYPE_FRACTION, 1, 1,
       "width", G_TYPE_INT, 640,
       "height", G_TYPE_INT, 480,
       "channels", G_TYPE_INT, 4,
       NULL);
       rclcpp_subscriber->initialized_caps = CapsState::DUMMY_CAPS;
       return GST_BASE_SRC_CLASS (gst_rclcpp_subscriber_parent_class)->fixate (src, new_caps);
  } else  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(rclcpp_subscriber->node);
    return caps;
  }
  g_assert(FALSE);
  return nullptr;
}
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

static bool gst_rclcpp_subscriber_need_updated_caps(
    GstRclcppSubscriber * src, const sensor_msgs::msg::Image& ros_image) {
  auto& srcpad = src->base_rclcppsubscriber.parent.srcpad;
  const GstCaps * caps  = gst_pad_get_current_caps(srcpad);
  GstStructure * structure = gst_caps_get_structure (caps, 0);
  gint new_width = 0;
  gint new_height = 0;
  gint new_channels = 0;
  std::string new_encoding;
  gst_rclcpp_subscriber_get_caps_info(ros_image, &new_width, &new_height, &new_encoding, &new_channels);

  gint height = 0;
  gint width = 0;
  gint channels = 0;
  const char* encoding = gst_structure_get_string(structure, "format");
  bool need_updating =
        encoding == nullptr
        || !gst_structure_get_int (structure, "width", &width)
        || !gst_structure_get_int (structure, "height", &height)
        || !gst_structure_get_int(structure, "channels", &channels)
        || new_width != width
        || new_height != height
        || new_channels != channels
        || strcmp(new_encoding.c_str(), encoding) != 0;
  if (need_updating) {
    std::cout << "New width: " << new_width << ", height: " << new_height
              << ", channels " << new_channels << ", encodings: " << new_encoding << std::endl;
    std::cout << "Previous width: " << width << ", height: " << height
              << ", channels " << channels << ", encodings: " << encoding << std::endl;
  }
  return need_updating;
}

static bool gst_rclcpp_subscriber_update_caps(
    GstRclcppSubscriber * src, const sensor_msgs::msg::Image& ros_image) {
  gint new_width = 0;
  gint new_height = 0;
  gint new_channels = 0;
  std::string new_encoding;
  gst_rclcpp_subscriber_get_caps_info(ros_image, &new_width, &new_height, &new_encoding, &new_channels);

  auto& srcpad = src->base_rclcppsubscriber.parent.srcpad;
  GstCaps * caps  = gst_caps_new_simple ("video/x-raw",
     "format", G_TYPE_STRING, new_encoding.c_str(),
     "framerate", GST_TYPE_FRACTION, 30, 1,
     "width", G_TYPE_INT, new_width,
     "height", G_TYPE_INT, new_height,
     "channels", G_TYPE_INT, new_channels,
     NULL);
  // caps = gst_caps_make_writable (caps);
  //GstStructure * structure = gst_caps_get_structure (caps, 0);
  // gst_rclcpp_subscriber_set_caps(caps, new_width, new_height, new_encoding.c_str(), new_channels);
  std::cout << "Setting new caps: " << std::endl;
  g_assert(caps);
  print_caps(caps, "    ");
  return gst_pad_set_caps(srcpad, caps);
  // GstEvent * new_caps_event = gst_event_new_caps(caps);
  // return gst_element_send_event(&src->base_rclcppsubscriber.parent.element, new_caps_event);
}

/* ask the subclass to fill the buffer with data from offset and size */
static GstFlowReturn
gst_rclcpp_subscriber_fill (GstPushSrc * src, GstBuffer * buf)
{
  GstRclcppSubscriber *rclcpp_subscriber = GST_RCLCPP_SUBSCRIBER (src);
  while (rclcpp_subscriber->queue->empty()) {
    rclcpp::spin_some(rclcpp_subscriber->node);
    g_print("No image, spinning\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    //return GST_FLOW_OK;
  }

  auto image = rclcpp_subscriber->queue->front();
  if (gst_rclcpp_subscriber_need_updated_caps(rclcpp_subscriber, *image)) {
    std::cout << "Incoming image different than negotiated caps, updating" << std::endl;
    if (!gst_rclcpp_subscriber_update_caps(rclcpp_subscriber, *image)) {
      g_print("Updating caps failed\n");
      return GST_FLOW_ERROR;
    }
    gst_pad_send_event (GST_BASE_SRC (src)->srcpad,
        gst_event_new_reconfigure ());
    return GST_FLOW_OK;
  }
  rclcpp_subscriber->queue->pop();

  GstMapInfo minfo;
  gst_buffer_map (buf, &minfo, GST_MAP_WRITE);
  if (image->data.size() != minfo.size) {
    std::cerr << "Incoming buffer doesn't match image size. "
              << image->data.size() << " != " << minfo.size << " max size: " << minfo.maxsize << std::endl;
    if (!gst_rclcpp_subscriber_update_caps(rclcpp_subscriber, *image)) {
      g_print("Updating caps failed\n");
      return GST_FLOW_ERROR;
    }
    gst_pad_send_event (GST_BASE_SRC (src)->srcpad,
        gst_event_new_reconfigure ());
    return GST_FLOW_OK;
  }
  size_t timestamp = ((image->header.stamp.sec * 1000000000u) % 1000) + image->header.stamp.nanosec;
  buf->pts = timestamp;//image->header.stamp.nanosec;
  buf->dts = GST_CLOCK_TIME_NONE;// image->header.stamp.nanosec;
  buf->duration = GST_CLOCK_TIME_NONE;
  memcpy(minfo.data, &image->data[0], image->data.size());
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

GstSubscriberNode::GstSubscriberNode(
  const std::string& name, std::shared_ptr<std::queue<sensor_msgs::msg::Image::ConstSharedPtr>> queue)
: rclcpp::Node(name),  queue_(queue) {}

GstSubscriberNode::~GstSubscriberNode() {}

void GstSubscriberNode::set_topic_name(const std::string& topic_name) {
  std::cout << "Subscribing to topic: " << topic_name << std::endl;
  subscriber_ = image_transport::create_subscription(
    this, topic_name, std::bind(&GstSubscriberNode::on_image, this, std::placeholders::_1), "raw");
}

void GstSubscriberNode::on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  while(queue_->size() > 1) {
    g_print("Dropping frames, maybe increase framerate?");
    queue_->pop();
  }
  auto now = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
  auto diff = now - prev_time_;
  // std::cout << "Diff: " << diff.nanoseconds()/1000 << std::endl;
  prev_time_ = now;
  queue_->push(msg);
}
