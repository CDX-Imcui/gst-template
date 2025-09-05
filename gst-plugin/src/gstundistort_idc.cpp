/**
 * gstundistort_idc.cpp
 *
 * 保持原始 undistort 元素的外部接口与属性（fx/fy/cx/cy/k1...）
 * 内部改为直接使用 Rockchip IDC（librkalg_idc）：
 *  - set_info: 用 OpenCV initUndistortRectifyMap 生成稠密 mapx/mapy（基于 element 属性）
 *              将 mapx,mapy 采样成 IDC 要求的稀疏 merged meshXY（内存，不写 bin）
 *              用 RKALG_IDC_LUT_Init 初始化 IDC 上下文
 *  - transform_frame_ip: 将 GST 的 NV12 frame 包装为 RKALG_IDC_IMAGE_S，调用 RKALG_IDC_LUT_DoLut
 *                        再把结果拷回原 frame（按行拷贝以兼容步长）
 *
 * 要求：输入必须是 NV12（video/x-raw,format=NV12）。如果你现在是 BGR 输入，
 * 在 pipeline 前插入 videoconvert ! video/x-raw,format=NV12 即可。
 *
 * 编译示例（在项目中按你原来的 meson/cmake 调整）：
 * g++ -fPIC -shared -o libgstundistort_idc.so gstundistort_idc.cpp \
 *   `pkg-config --cflags --libs gstreamer-1.0 gstreamer-video-1.0 opencv4` \
 *   -I/path/to/rk/include -L/path/to/rk/lib -lrkalg_idc
 *
 * 示例 pipeline:
 * gst-launch-1.0 v4l2src device=/dev/video0 ! jpegdec ! videoconvert \
 *   ! video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 \
 *   ! undistort fx=619.97 fy=625.27 cx=586.32 cy=339.90 k1=-0.291149 k2=0.057760 p1=-0.006811 p2=0.001601 k3=0.0 \
 *   ! videoconvert ! x265enc ... ! rtspclientsink ...
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>
#include <gst/video/gstvideoframe.h>
#include "gstundistort.h"

#include <opencv2/opencv.hpp>
#include "rkalg_idc_lut_api.h"

#include <cstring>
#include <cstdlib>
#include <memory>

GST_DEBUG_CATEGORY_STATIC(gst_undistort_debug);
#define GST_CAT_DEFAULT gst_undistort_debug

using namespace cv;

/* 私有数据：保存 IDC 上下文、mesh 与对齐缓冲区 */
typedef struct _GstUndistortPrivate {
    GstVideoInfo info;
    // IDC context
    RKALG_LUT_CTX_S lutCtx;
    // merged mesh (float x,y interleaved)
    float *mesh_xy; // length meshW*meshH*2
    uint32_t meshW;
    uint32_t meshH;
    uint32_t stepX;
    uint32_t stepY;
    // dst aligned nv12 buffer for IDC output
    unsigned char *dst_nv12;
    uint32_t dst_stride;
    uint32_t dst_hstride;
    gboolean maps_ready;
    gboolean idc_inited;
} GstUndistortPrivate;

/* 属性与信号枚举（和你原来的一样） */
enum {
    PROP_0,
    PROP_SILENT,
    PROP_FX, PROP_FY, PROP_CX, PROP_CY,
    PROP_K1, PROP_K2, PROP_P1, PROP_P2, PROP_K3,
};

/* Pad 模板：现在要求 NV12（IDC 要求） */
static GstStaticPadTemplate sink_template_video =
        GST_STATIC_PAD_TEMPLATE("sink",
                                GST_PAD_SINK, GST_PAD_ALWAYS,
                                GST_STATIC_CAPS ("video/x-raw, format=(string)NV12")
        );

static GstStaticPadTemplate src_template_video =
        GST_STATIC_PAD_TEMPLATE("src",
                                GST_PAD_SRC, GST_PAD_ALWAYS,
                                GST_STATIC_CAPS ("video/x-raw, format=(string)NV12")
        );

/* 保持原始类型定义宏与注册名（最小改动） */
#define gst_undistort_parent_class parent_class
G_DEFINE_TYPE_WITH_PRIVATE(GstUndistort, gst_undistort, GST_TYPE_VIDEO_FILTER);
#if GST_CHECK_VERSION(1, 20, 0)
GST_ELEMENT_REGISTER_DEFINE(undistort, "undistort", GST_RANK_NONE, GST_TYPE_UNDISTORT);
#endif

/* 前置声明（名字与原来完全一致） */
static void gst_undistort_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);
static void gst_undistort_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);
static gboolean gst_undistort_set_info(GstVideoFilter *filter,
                                       GstCaps *incaps, GstVideoInfo *in_info,
                                       GstCaps *outcaps, GstVideoInfo *out_info);
static GstFlowReturn gst_undistort_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame);
static void gst_undistort_finalize(GObject *object);

/* 原来私有数据里有 cameraMatrix,mapx,mapy 等，改为 IDC 相关字段 */
static void
gst_undistort_class_init(GstUndistortClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstElementClass *gstelement_class = GST_ELEMENT_CLASS(klass);
    GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

    gobject_class->set_property = gst_undistort_set_property;
    gobject_class->get_property = gst_undistort_get_property;
    gobject_class->finalize = gst_undistort_finalize;

    /* 属性：保持原有属性（不变） */
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
                                         "Undistort video frames using Rockchip IDC (mesh generated in-memory)",
                                         "you <you@example.com>");

    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&src_template_video));
    gst_element_class_add_pad_template(gstelement_class,
                                       gst_static_pad_template_get(&sink_template_video));

    vfilter_class->set_info = GST_DEBUG_FUNCPTR(gst_undistort_set_info);
    vfilter_class->transform_frame_ip = GST_DEBUG_FUNCPTR(gst_undistort_transform_frame_ip);

    GST_DEBUG_CATEGORY_INIT(gst_undistort_debug, "undistort", 0, "Undistort filter using Rockchip IDC");
}

/* init：设默认值（保留原属性默认） */
static void
gst_undistort_init(GstUndistort *self) {
    self->silent = FALSE;
    self->fx = self->fy = self->cx = self->cy = 0.0;
    self->k1 = self->k2 = self->p1 = self->p2 = self->k3 = 0.0;

    GstUndistortPrivate *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);
    memset(priv, 0, sizeof(GstUndistortPrivate));
    priv->mesh_xy = nullptr;
    priv->dst_nv12 = nullptr;
    priv->maps_ready = FALSE;
    priv->idc_inited = FALSE;
    // default sampling step (可按需改或暴露为属性)
    priv->stepX = 16;
    priv->stepY = 8;
}

/* 属性读写（保持原样） */
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

/* helper: alignment */
static inline uint32_t align_up(uint32_t v, uint32_t a) {
    return (v + a - 1) & ~(a - 1);
}

/* 将 dense mapx/mapy 转换成 IDC merged meshXY（内存）——来自 Rockchip 文档逻辑 */
static void convertDenseMapToIDCMeshXY(
    int dstW, int dstH,
    int meshW, int meshH,
    int stepX, int stepY,
    const float *pf32mapx, const float *pf32mapy,
    float *pMeshXY)
{
    for (int row = 0, mesh_row = 0; mesh_row < meshH; row += stepY, mesh_row++) {
        for (int col = 0, mesh_col = 0; mesh_col < meshW; col += stepX, mesh_col++) {
            size_t mesh_idx = (mesh_row * meshW + mesh_col) * 2;
            if (col >= dstW) {    /* Right border extrapolate */
                int last_sampled_col = col;
                while (last_sampled_col >= dstW) last_sampled_col -= stepX;
                int a = (dstW - 1) - last_sampled_col;
                int b = col - (dstW - 1);
                float x_last_sampled = pMeshXY[2 * (last_sampled_col / stepX + meshW * mesh_row)];
                float y_last_sampled = pMeshXY[2 * (last_sampled_col / stepX + meshW * mesh_row) + 1];
                size_t map_idx = std::min(row, dstH - 1) * dstW + (dstW - 1);
                float x_last_img = pf32mapx[map_idx];
                float y_last_img = pf32mapy[map_idx];
                float x_extrapolated = (stepX * x_last_img - b * x_last_sampled) / a;
                float y_extrapolated = (stepX * y_last_img - b * y_last_sampled) / a;
                pMeshXY[mesh_idx] = x_extrapolated;
                pMeshXY[mesh_idx + 1] = y_extrapolated;
                continue;
            }
            if (row >= dstH) {  /* Bottom border extrapolate */
                int last_sampled_row = row;
                while (last_sampled_row >= dstH) last_sampled_row -= stepY;
                int a = (dstH - 1) - last_sampled_row;
                int b = row - (dstH - 1);
                float x_last_sampled = pMeshXY[2 * (mesh_col + meshW * (last_sampled_row / stepY))];
                float y_last_sampled = pMeshXY[2 * (mesh_col + meshW * (last_sampled_row / stepY)) + 1];
                float x_last_img = pf32mapx[(dstH - 1) * dstW + col];
                float y_last_img = pf32mapy[(dstH - 1) * dstW + col];
                float x_extrapolated = (stepY * x_last_img - b * x_last_sampled) / a;
                float y_extrapolated = (stepY * y_last_img - b * y_last_sampled) / a;
                pMeshXY[mesh_idx] = x_extrapolated;
                pMeshXY[mesh_idx + 1] = y_extrapolated;
                continue;
            }
            size_t map_idx = row * dstW + col;
            pMeshXY[mesh_idx] = pf32mapx[map_idx];
            pMeshXY[mesh_idx + 1] = pf32mapy[map_idx];
        }
    }
}

/* set_info：初始化 OpenCV dense map -> 生成 IDC mesh -> 初始化 IDC 上下文 */
static gboolean
gst_undistort_set_info(GstVideoFilter *filter,
                       GstCaps *incaps, GstVideoInfo *in_info,
                       GstCaps *outcaps, GstVideoInfo *out_info) {
    auto *self = GST_UNDISTORT(filter);
    auto *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);

    priv->info = *in_info;

    const int w = GST_VIDEO_INFO_WIDTH(&priv->info);
    const int h = GST_VIDEO_INFO_HEIGHT(&priv->info);

    /* 如果没设置内参，就旁路（保持原行为） */
    if (self->fx <= 0 || self->fy <= 0) {
        GST_WARNING_OBJECT(self, "fx/fy not set, bypassing undistortion (identity).");
        priv->maps_ready = FALSE;
        return TRUE;
    }

    // ===== 使用 OpenCV 生成稠密 mapx/mapy（和原来一致） =====
    Mat cameraMatrix = (Mat_<double>(3, 3) << self->fx, 0, self->cx, 0, self->fy, self->cy, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(1, 5) << self->k1, self->k2, self->p1, self->p2, self->k3);

    Mat mapx, mapy; // CV_32FC1
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            cameraMatrix, Size(w, h),
                            CV_32FC1, mapx, mapy);

    // ===== 将稠密 map 转为 IDC merged meshXY（内存） =====
    // stepX/stepY 采用 priv 默认（16/8），也可以改成根据分辨率自动选择
    uint32_t stepX = priv->stepX;
    uint32_t stepY = priv->stepY;
    uint32_t meshW = (w - 1) / stepX + 2;
    uint32_t meshH = (h - 1) / stepY + 2;
    size_t meshElems = (size_t)meshW * (size_t)meshH * 2; // x,y

    // 释放旧 mesh，重分配
    if (priv->mesh_xy) {
        free(priv->mesh_xy);
        priv->mesh_xy = nullptr;
    }
    priv->mesh_xy = (float *)malloc(meshElems * sizeof(float));
    if (!priv->mesh_xy) {
        GST_ERROR_OBJECT(self, "Failed to allocate mesh_xy (%zu floats).", meshElems);
        return FALSE;
    }
    priv->meshW = meshW;
    priv->meshH = meshH;
    priv->stepX = stepX;
    priv->stepY = stepY;

    // mapx,mapy 内存连续，类型 float
    convertDenseMapToIDCMeshXY(w, h, meshW, meshH, stepX, stepY,
                               (const float *)mapx.ptr<float>(0),
                               (const float *)mapy.ptr<float>(0),
                               priv->mesh_xy);

    // ===== 准备并调用 RKALG_IDC_LUT_Init（IDC 上下文） =====
    RKALG_LUT_INIT_PARAMS_S stInit;
    memset(&stInit, 0, sizeof(stInit));
    stInit.u32SrcWidth = w;
    stInit.u32SrcHeight = h;
    stInit.u32SrcStride = align_up((uint32_t)w, 64);   // 按 demo 建议对齐
    stInit.u32SrcHgtStride = align_up((uint32_t)h, 2);
    stInit.u32DstWidth = w;
    stInit.u32DstHeight = h;
    stInit.u32DstStride = align_up((uint32_t)w, 16);
    stInit.u32DstHgtStride = align_up((uint32_t)h, 8);
    stInit.eMode = RKALG_IDC_LUT_DEFAULT_MODE;

    int ret = RKALG_IDC_LUT_Init(&priv->lutCtx, &stInit);
    if (ret != 0) {
        GST_ERROR_OBJECT(self, "RKALG_IDC_LUT_Init failed: %d", ret);
        free(priv->mesh_xy);
        priv->mesh_xy = nullptr;
        return FALSE;
    }
    priv->idc_inited = TRUE;

    // ===== 分配对齐的 dst buffer（NV12）以供 IDC 写入 =====
    priv->dst_stride = stInit.u32DstStride;
    priv->dst_hstride = stInit.u32DstHgtStride;
    size_t y_area = (size_t)priv->dst_stride * (size_t)priv->dst_hstride;
    size_t uv_area = (size_t)priv->dst_stride * (size_t)(priv->dst_hstride / 2);
    size_t total_size = y_area + uv_area;
    // 对齐分配，4096 较安全；在 RK 板上建议用 alloc_drm_buf 替换
    if (priv->dst_nv12) {
        free(priv->dst_nv12);
        priv->dst_nv12 = nullptr;
    }
    if (posix_memalign((void **)&priv->dst_nv12, 4096, total_size) != 0) {
        GST_ERROR_OBJECT(self, "posix_memalign for dst_nv12 failed");
        RKALG_IDC_LUT_Deinit(&priv->lutCtx);
        priv->idc_inited = FALSE;
        free(priv->mesh_xy);
        priv->mesh_xy = nullptr;
        return FALSE;
    }
    memset(priv->dst_nv12, 0, total_size);

    priv->maps_ready = TRUE;
    if (!self->silent) {
        GST_INFO_OBJECT(self, "Prepared IDC mesh (%u x %u), dst stride=%u/hgtstride=%u.", priv->meshW, priv->meshH, priv->dst_stride, priv->dst_hstride);
    }

    return TRUE;
}

/* transform_frame_ip: 用 IDC 做实际重映射；输入假定 NV12（在 pipeline 外转换）替换掉opencv::remap */
static GstFlowReturn
gst_undistort_transform_frame_ip(GstVideoFilter *filter, GstVideoFrame *frame) {
    auto *self = GST_UNDISTORT(filter);
    auto *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);

    if (!priv->maps_ready || !priv->mesh_xy || !priv->idc_inited) {
        return GST_FLOW_OK;
    }

    const int w = GST_VIDEO_FRAME_WIDTH(frame);
    const int h = GST_VIDEO_FRAME_HEIGHT(frame);

    // 获取 NV12 平面数据
    guint8 *y_src = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
    guint8 *uv_src = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 1));
    int y_stride_src = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
    int uv_stride_src = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 1);

    // 准备 RKALG_IDC_IMAGE_S src (NV12)
    RKALG_IDC_IMAGE_S srcImg;
    memset(&srcImg, 0, sizeof(srcImg));
    srcImg.eImgFmt = RKALG_IDC_IMG_FMT_NV12;
    srcImg.u32Width = w;
    srcImg.u32Height = h;
    srcImg.u32Stride[0] = (uint32_t)y_stride_src;
    srcImg.u32HgtStride[0] = (uint32_t)h;
    srcImg.virAddr[0] = (void *)y_src;
    srcImg.virAddr[1] = (void *)uv_src;

    // 准备 RKALG_IDC_IMAGE_S dst（指向对齐缓冲区）
    RKALG_IDC_IMAGE_S dstImg;
    memset(&dstImg, 0, sizeof(dstImg));
    dstImg.eImgFmt = RKALG_IDC_IMG_FMT_NV12;
    dstImg.u32Width = w;
    dstImg.u32Height = h;
    dstImg.u32Stride[0] = priv->dst_stride;
    dstImg.u32HgtStride[0] = priv->dst_hstride;
    dstImg.virAddr[0] = (void *)priv->dst_nv12;
    dstImg.virAddr[1] = (void *)(priv->dst_nv12 + (size_t)priv->dst_stride * (size_t)priv->dst_hstride);

    // 准备 RKALG_IDC_MESH_S（指向内存 mesh_xy）
    RKALG_IDC_MESH_S mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.u32StepX = priv->stepX;
    mesh.u32StepY = priv->stepY;
    mesh.u32Width = priv->meshW;
    mesh.u32Height = priv->meshH;
    mesh.u32Stride = priv->meshW;
    mesh.u32HgtStride = priv->meshH;
    mesh.eMeshType = RKALG_IDC_MESH_TYPE_MERGED;
    mesh.virAddr[0] = priv->mesh_xy;

    // 组装并调用 RKALG_IDC_LUT_DoLut
    RKALG_LUT_TASK_S task;
    memset(&task, 0, sizeof(task));
    task.pSrcImage = &srcImg;
    task.pDstImage = &dstImg;
    task.pMesh = &mesh;
    task.pOpAttr = nullptr; // default mode

    int rc = RKALG_IDC_LUT_DoLut(&priv->lutCtx, &task);
    if (rc != 0) {
        GST_WARNING_OBJECT(self, "RKALG_IDC_LUT_DoLut failed: %d", rc);
        return GST_FLOW_OK; // 失败则不破坏 frame
    }

    // 把对齐的 dst_buf 拷回 frame（按行拷贝，兼容不同 stride）
    uint8_t *dst_y_base = (uint8_t *)dstImg.virAddr[0];
    uint8_t *dst_uv_base = (uint8_t *)dstImg.virAddr[1];

    for (int row = 0; row < h; ++row) {
        memcpy(y_src + row * y_stride_src, dst_y_base + row * priv->dst_stride, (size_t)w);
    }
    int uv_h = h / 2;
    for (int row = 0; row < uv_h; ++row) {
        memcpy(uv_src + row * uv_stride_src, dst_uv_base + row * priv->dst_stride, (size_t)w);
    }

    return GST_FLOW_OK;
}

/* finalize：释放 IDC 上下文与缓冲 */
static void gst_undistort_finalize(GObject *object) {
    GstUndistort *self = GST_UNDISTORT(object);
    GstUndistortPrivate *priv = (GstUndistortPrivate *) gst_undistort_get_instance_private(self);

    if (priv->idc_inited) {
        RKALG_IDC_LUT_Deinit(&priv->lutCtx);
        priv->idc_inited = FALSE;
    }
    if (priv->mesh_xy) {
        free(priv->mesh_xy);
        priv->mesh_xy = nullptr;
    }
    if (priv->dst_nv12) {
        free(priv->dst_nv12);
        priv->dst_nv12 = nullptr;
    }
    G_OBJECT_CLASS(parent_class)->finalize(object);
}

/* 插件初始化：注册元素（保留原名 undistort） */
static gboolean
undistort_init(GstPlugin *plugin) {
#if GST_CHECK_VERSION(1, 20, 0)
    return GST_ELEMENT_REGISTER(undistort, plugin);
#else
    return gst_element_register(plugin, "undistort", GST_RANK_NONE, GST_TYPE_UNDISTORT);
#endif
}

#ifndef PACKAGE
#define PACKAGE "gst-undistort"
#endif

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR,
                  undistort, "Undistort filter (IDC, in-memory mesh)",
                  undistort_init,
                  "1.0", "LGPL", "gst-undistort", "https://example.org/")
