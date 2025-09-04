/**
 * SECTION:element-undistort
 *
 * Undistort video frames using OpenCV remap.
 *
 * Example:
  gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! video/x-raw,format=BGR ! undistort fx=800 fy=800 cx=640 cy=360 k1=-0.2 k2=0.1 p1=0.0 p2=0.0 k3=0.0  ! videoconvert !  x265enc bitrate=1800 speed-preset=ultrafast tune=zerolatency ! rtspclientsink location=rtsp://127.0.0.1:8554/video1 latency=10

*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>
#include "gstundistort.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

GST_DEBUG_CATEGORY_STATIC(gst_undistort_debug);
#define GST_CAT_DEFAULT gst_undistort_debug

using namespace cv;

/* 私有数据：OpenCV 矩阵与映射表 */
typedef struct _GstUndistortPrivate {
    GstVideoInfo info;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat mapx, mapy; // CV_32FC1
    cv::Mat scratch; // 复用的临时图像，避免频繁分配
    gboolean maps_ready;
} GstUndistortPrivate;

/* 属性与信号枚举 */
enum {
    PROP_0,
    PROP_SILENT,
    PROP_FX, PROP_FY, PROP_CX, PROP_CY,
    PROP_K1, PROP_K2, PROP_P1, PROP_P2, PROP_K3,
};

/* Pad 模板（BGR 8UC3，更贴 OpenCV；若要支持更多格式，先接 videoconvert） */
static GstStaticPadTemplate sink_template_video =
        GST_STATIC_PAD_TEMPLATE("sink",
                                GST_PAD_SINK, GST_PAD_ALWAYS,
                                GST_STATIC_CAPS ("video/x-raw, format=(string)BGR")
        );

static GstStaticPadTemplate src_template_video =
        GST_STATIC_PAD_TEMPLATE("src",
                                GST_PAD_SRC, GST_PAD_ALWAYS,
                                GST_STATIC_CAPS ("video/x-raw, format=(string)BGR")
        );

/* 类型定义：使用带私有数据的宏 */
#define gst_undistort_parent_class parent_class
G_DEFINE_TYPE_WITH_PRIVATE(GstUndistort, gst_undistort, GST_TYPE_VIDEO_FILTER);
#if GST_CHECK_VERSION(1, 20, 0)
//生成元素类型的注册函数,将自定义元素undistort注册到 GStreamer 框架，使其可以被管道识别和使用,
GST_ELEMENT_REGISTER_DEFINE(undistort, "undistort", GST_RANK_NONE, GST_TYPE_UNDISTORT);
#endif

/* 前置声明 */
static void gst_undistort_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);

static void gst_undistort_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);

static gboolean gst_undistort_set_info(GstVideoFilter *filter,
                                       GstCaps *incaps, GstVideoInfo *in_info,
                                       GstCaps *outcaps, GstVideoInfo *out_info);

static GstFlowReturn gst_undistort_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame);

/* class_init：注册属性/回调/Pad 与元信息 */
static void
gst_undistort_class_init(GstUndistortClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass *gstelement_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    gobject_class->set_property = gst_undistort_set_property;
    gobject_class->get_property = gst_undistort_get_property;

    /* 属性：silent 与相机参数，默认 0 表示“不做畸变校正”（生成单位映射） */
    g_object_class_install_property(gobject_class, PROP_SILENT,
                                    g_param_spec_boolean("silent", "Silent", "Reduce verbose output",
                                                         FALSE,
                                                         (GParamFlags) (G_PARAM_READWRITE | GST_PARAM_CONTROLLABLE)));

    g_object_class_install_property(gobject_class, PROP_FX,
                                    g_param_spec_double("fx", "fx", "Focal length fx (pixels)", 0.0, G_MAXDOUBLE, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_FY,
                                    g_param_spec_double("fy", "fy", "Focal length fy (pixels)", 0.0, G_MAXDOUBLE, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_CX,
                                    g_param_spec_double("cx", "cx", "Principal point cx", 0.0, G_MAXDOUBLE, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_CY,
                                    g_param_spec_double("cy", "cy", "Principal point cy", 0.0, G_MAXDOUBLE, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_K1,
                                    g_param_spec_double("k1", "k1", "Radial distortion k1", -10.0, 10.0, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_K2,
                                    g_param_spec_double("k2", "k2", "Radial distortion k2", -10.0, 10.0, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_P1,
                                    g_param_spec_double("p1", "p1", "Tangential distortion p1", -10.0, 10.0, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_P2,
                                    g_param_spec_double("p2", "p2", "Tangential distortion p2", -10.0, 10.0, 0.0,
                                                        G_PARAM_READWRITE));
    g_object_class_install_property(gobject_class, PROP_K3,
                                    g_param_spec_double("k3", "k3", "Radial distortion k3", -10.0, 10.0, 0.0,
                                                        G_PARAM_READWRITE));

    gst_element_class_set_details_simple(gstelement_class,
                                         "Undistort", "Filter/Video",
                                         "Undistort video frames using OpenCV remap",
                                         "you <you@example.com>");

    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&src_template_video));
    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&sink_template_video));

    vfilter_class->set_info = GST_DEBUG_FUNCPTR(gst_undistort_set_info);
    vfilter_class->transform_frame_ip = GST_DEBUG_FUNCPTR(gst_undistort_transform_frame_ip);

    GST_DEBUG_CATEGORY_INIT(gst_undistort_debug, "undistort", 0, "Undistort filter");
}

/* init：设默认值 */
static void
gst_undistort_init(GstUndistort *self) {
    self->silent = FALSE;
    self->fx = self->fy = self->cx = self->cy = 0.0;
    self->k1 = self->k2 = self->p1 = self->p2 = self->k3 = 0.0;

    auto *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);
    priv->maps_ready = FALSE;
}

/* 属性读写 */
static void
gst_undistort_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec) {
    GstUndistort *self = GST_UNDISTORT(object);
    switch (prop_id) {
        case PROP_SILENT: self->silent = g_value_get_boolean(value);
            break;
        case PROP_FX: self->fx = g_value_get_double(value);
            break;
        case PROP_FY: self->fy = g_value_get_double(value);
            break;
        case PROP_CX: self->cx = g_value_get_double(value);
            break;
        case PROP_CY: self->cy = g_value_get_double(value);
            break;
        case PROP_K1: self->k1 = g_value_get_double(value);
            break;
        case PROP_K2: self->k2 = g_value_get_double(value);
            break;
        case PROP_P1: self->p1 = g_value_get_double(value);
            break;
        case PROP_P2: self->p2 = g_value_get_double(value);
            break;
        case PROP_K3: self->k3 = g_value_get_double(value);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

static void
gst_undistort_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec) {
    GstUndistort *self = GST_UNDISTORT(object);
    switch (prop_id) {
        case PROP_SILENT: g_value_set_boolean(value, self->silent);
            break;
        case PROP_FX: g_value_set_double(value, self->fx);
            break;
        case PROP_FY: g_value_set_double(value, self->fy);
            break;
        case PROP_CX: g_value_set_double(value, self->cx);
            break;
        case PROP_CY: g_value_set_double(value, self->cy);
            break;
        case PROP_K1: g_value_set_double(value, self->k1);
            break;
        case PROP_K2: g_value_set_double(value, self->k2);
            break;
        case PROP_P1: g_value_set_double(value, self->p1);
            break;
        case PROP_P2: g_value_set_double(value, self->p2);
            break;
        case PROP_K3: g_value_set_double(value, self->k3);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    }
}

/* 在协商阶段初始化 VideoInfo 并预计算 remap 映射表 */
static gboolean
gst_undistort_set_info(GstVideoFilter *filter,
                       GstCaps *incaps, GstVideoInfo *in_info,
                       GstCaps *outcaps, GstVideoInfo *out_info) {
    auto *self = GST_UNDISTORT(filter);
    auto *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);

    priv->info = *in_info;

    const int w = GST_VIDEO_INFO_WIDTH(&priv->info);
    const int h = GST_VIDEO_INFO_HEIGHT(&priv->info);

    /* 如果没设置内参，就退化为“恒等映射”（不做矫正） */
    if (self->fx <= 0 || self->fy <= 0) {
        GST_WARNING_OBJECT(self, "fx/fy not set, bypassing undistortion (identity map).");
        priv->mapx.release();
        priv->mapy.release();
        priv->maps_ready = FALSE;
        return TRUE;
    }

    /* 准备 K/D 并生成映射表（CV_32FC1） */
    priv->cameraMatrix = (cv::Mat_<double>(3, 3) << self->fx, 0, self->cx,
                          0, self->fy, self->cy,
                          0, 0, 1);
    priv->distCoeffs = (cv::Mat_<double>(1, 5) << self->k1, self->k2, self->p1, self->p2, self->k3);

    cv::initUndistortRectifyMap(priv->cameraMatrix, priv->distCoeffs, cv::Mat(),
                                priv->cameraMatrix, cv::Size(w, h),
                                CV_32FC1, priv->mapx, priv->mapy);
    priv->scratch.release(); /* 将在第一帧按需分配 */
    priv->maps_ready = TRUE;

    if (!self->silent) {
        GST_INFO_OBJECT(self, "Prepared undistort maps (%dx%d).", w, h);
    }

    if (cv::ocl::haveOpenCL()) {
        cv::ocl::setUseOpenCL(true);//检测到系统支持 OpenCL 时自动启用 ，前提是 OpenCV 编译时已启用 OpenCL 支持
    }
    return TRUE;
}

/* 帧内把 frame 映射成 cv::Mat，remap 到临时图，再拷回 */
static GstFlowReturn
gst_undistort_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame) {
    auto *self = GST_UNDISTORT(filter);
    auto *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);

    const int w = GST_VIDEO_FRAME_WIDTH(frame);
    const int h = GST_VIDEO_FRAME_HEIGHT(frame);
    guint8 *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
    const int stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);

    /* 当 maps 不可用时直接旁路（比如没设置 fx/fy） */
    if (!priv->maps_ready || priv->mapx.empty() || priv->mapy.empty()) {
        return GST_FLOW_OK;
    }

    /* OpenCV 视图：注意 stride */
    cv::Mat img(h, w, CV_8UC3, data, (size_t) stride);

    if (priv->scratch.empty() || priv->scratch.cols != w || priv->scratch.rows != h)
        priv->scratch.create(h, w, CV_8UC3);

    // CV_16SC2只适合INTER_NEAREST插值方式
    // CV_32FC1可适合INTER_LINEAR插值方式，但会慢一些
    cv::remap(img, priv->scratch, priv->mapx, priv->mapy, cv::INTER_LINEAR); //INTER_LINEAR
    std::memcpy(data, priv->scratch.data, (size_t) h * stride);
    // /* 若步长一致可整块 memcpy，否则逐行 */
    // if ((int)priv->scratch.step[0] == stride) {
    //   std::memcpy(data, priv->scratch.data, (size_t)h * stride);
    // } else {
    //   for (int y = 0; y < h; ++y) {
    //     std::memcpy(data + y * stride, priv->scratch.ptr(y), (size_t)w * 3);
    //   }
    // }

    // if (!self->silent) {
    //   GST_LOG_OBJECT (self, "undistort applied.");
    // }
    return GST_FLOW_OK;
}

/* 插件初始化：注册元素 */
static gboolean
undistort_init(GstPlugin *plugin) {
#if GST_CHECK_VERSION(1, 20, 0)
    return GST_ELEMENT_REGISTER(undistort, plugin); //自动使用 GST_ELEMENT_REGISTER_DEFINE 生成的注册函数
#else//旧版本的标准注册接口，需要指定元素名称、优先级和类型
    return gst_element_register(plugin, "undistort", GST_RANK_NONE, GST_TYPE_UNDISTORT);
#endif
}

#ifndef PACKAGE
#define PACKAGE "gst-undistort"
#endif
/* 插件定义宏 */
GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR,
                  undistort, "Undistort filter created by cuidongxu",
                  undistort_init,
                  //PACKAGE_VERSION, GST_LICENSE, GST_PACKAGE_NAME, GST_PACKAGE_ORIGIN)
                  "1.0", "LGPL", "gst-undistort", "https://example.org/"
)
