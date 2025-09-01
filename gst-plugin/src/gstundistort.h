#ifndef __GST_UNDISTORT_H__
#define __GST_UNDISTORT_H__

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>
G_BEGIN_DECLS

#define GST_TYPE_UNDISTORT            (gst_undistort_get_type())
#define GST_UNDISTORT(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_UNDISTORT,GstUndistort))
#define GST_UNDISTORT_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_UNDISTORT,GstUndistortClass))
#define GST_IS_UNDISTORT(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_UNDISTORT))
#define GST_IS_UNDISTORT_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_UNDISTORT))

typedef struct _GstUndistort        GstUndistort;
typedef struct _GstUndistortClass   GstUndistortClass;
typedef struct _GstUndistort {
    GstVideoFilter parent;
    /* 简单属性：静默与相机内参/畸变系数 */
    gboolean silent;
    gdouble fx, fy, cx, cy;   /* 内参 */
    gdouble k1, k2, p1, p2, k3; /* 畸变系数（径向 k1/k2/k3 + 切向 p1/p2） */
} GstUndistort;

typedef struct _GstUndistortClass {
    GstVideoFilterClass parent_class;
} GstUndistortClass;

GType gst_undistort_get_type (void);
GST_ELEMENT_REGISTER_DECLARE (undistort)

G_END_DECLS
#endif /* __GST_UNDISTORT_H__ */
