//
// Created by CDX on 2025/8/31.
//
#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

// 定义自定义元素类
typedef struct _GstUndistort {
    GstVideoFilter parent;
    Mat cameraMatrix;
    Mat distCoeffs;
} GstUndistort;

typedef struct _GstUndistortClass {
    GstVideoFilterClass parent_class;
} GstUndistortClass;

G_DEFINE_TYPE(GstUndistort, gst_undistort, GST_TYPE_VIDEO_FILTER);

// 转换函数：对每一帧视频做去畸变
static GstFlowReturn gst_undistort_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame) {
    GstUndistort *ud = (GstUndistort *)filter;

    // 将 GStreamer buffer 映射为 OpenCV Mat
    Mat img(Size(GST_VIDEO_FRAME_WIDTH(frame), GST_VIDEO_FRAME_HEIGHT(frame)),
            CV_8UC3, GST_VIDEO_FRAME_PLANE_DATA(frame, 0),
            GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0));

    // 输出去畸变结果
    Mat undistorted;
    undistort(img, undistorted, ud->cameraMatrix, ud->distCoeffs);

    // 将 undistorted 写回原始 buffer（注意大小一致）
    if (!undistorted.empty() && undistorted.size() == img.size()) {
        undistorted.copyTo(img);
    }

    return GST_FLOW_OK;
}

// 类初始化
static void gst_undistort_class_init(GstUndistortClass *klass) {
    GstVideoFilterClass *video_filter_class = GST_VIDEO_FILTER_CLASS(klass);
    video_filter_class->transform_frame_ip = gst_undistort_transform_frame_ip;
}

// 对象初始化
static void gst_undistort_init(GstUndistort *ud) {
    // 假设你事先知道相机内参和畸变参数
    ud->cameraMatrix = (Mat_<double>(3, 3) <<
                        800, 0, 640,
                        0, 800, 360,
                        0, 0, 1);
    ud->distCoeffs = (Mat_<double>(1, 5) << -0.2, 0.1, 0, 0, 0);
}

// 插件入口：注册元素
extern "C" gboolean gst_undistort_plugin_init(GstPlugin *plugin) {
    return gst_element_register(plugin, "undistort", GST_RANK_NONE, gst_undistort_get_type());
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    undistort,
    "Undistort video using OpenCV",
    gst_undistort_plugin_init,
    "1.0",
    "LGPL",
    "GStreamer",
    "https://gstreamer.freedesktop.org/"
)
