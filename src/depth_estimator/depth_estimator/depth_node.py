#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import message_filters

def _make_pc2(header, xyzrgb):
    fields = [
        PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    r = xyzrgb[:,3].astype(np.uint8); g = xyzrgb[:,4].astype(np.uint8); b = xyzrgb[:,5].astype(np.uint8)
    rgb_packed = (r.astype(np.uint32) << 16 | g.astype(np.uint32) << 8 | b.astype(np.uint32))
    rgb_f = rgb_packed.view(np.float32)
    data = np.column_stack([xyzrgb[:,:3], rgb_f]).astype(np.float32)
    pc = PointCloud2(header=header, height=1, width=data.shape[0], fields=fields,
                     is_bigendian=False, point_step=16, row_step=16*data.shape[0],
                     data=data.tobytes(), is_dense=True)
    return pc

def ransac_plane(pts, n_iter=100, thr=0.05):
    best = np.zeros(len(pts), dtype=bool); rng = np.random.default_rng(0)
    for _ in range(n_iter):
        idx = rng.choice(len(pts), 3, replace=False); p1,p2,p3 = pts[idx]
        n = np.cross(p2-p1, p3-p1); nl = np.linalg.norm(n)
        if nl < 1e-6: continue
        n /= nl; d = -np.dot(n,p1)
        mask = np.abs(pts @ n + d) < thr
        if mask.sum() > best.sum(): best = mask
    return best

class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__("depth_estimator")
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("ransac_iters", 80)
        self.declare_parameter("ransac_dist_thresh", 0.06)
        self._md  = self.get_parameter("max_depth_m").value
        self._vox = self.get_parameter("voxel_size").value
        self._ni  = self.get_parameter("ransac_iters").value
        self._dt  = self.get_parameter("ransac_dist_thresh").value
        self.bridge = CvBridge(); self._K = None
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self._sr = message_filters.Subscriber(self, Image, "/camera/color/image", qos_profile=be)
        self._sd = message_filters.Subscriber(self, Image, "/camera/depth/image", qos_profile=be)
        self._si = message_filters.Subscriber(self, CameraInfo, "/camera/color/camera_info", qos_profile=be)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sr, self._sd, self._si], queue_size=5, slop=0.05)
        self._sync.registerCallback(self.cb)
        self.pub_raw = self.create_publisher(PointCloud2, "/perception/points_raw", rel)
        self.pub_gnd = self.create_publisher(PointCloud2, "/perception/points_ground", rel)
        self.pub_obs = self.create_publisher(PointCloud2, "/perception/points_obs", rel)
        self.get_logger().info("DepthEstimatorNode ready.")

    def cb(self, rgb_msg, depth_msg, info_msg):
        if self._K is None:
            self._K = np.array(info_msg.k, dtype=np.float64).reshape(3,3)
        t0 = time.perf_counter()
        try:
            rgb   = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().warn(str(e)); return
        if depth.dtype == np.uint16: depth = depth.astype(np.float32)/1000.0
        depth = depth.astype(np.float32)
        h,w = depth.shape; fx,fy = self._K[0,0],self._K[1,1]; cx,cy = self._K[0,2],self._K[1,2]
        uu,vv = np.meshgrid(np.arange(w,dtype=np.float32), np.arange(h,dtype=np.float32))
        valid = np.isfinite(depth) & (depth > 0.1) & (depth < self._md)
        z = depth[valid]; x = (uu[valid]-cx)*z/fx; y = (vv[valid]-cy)*z/fy
        r = rgb[:,:,0][valid].astype(np.float32)
        g = rgb[:,:,1][valid].astype(np.float32)
        b = rgb[:,:,2][valid].astype(np.float32)
        xyzrgb = np.column_stack([x,y,z,r,g,b])
        if self._vox > 0 and len(xyzrgb):
            c = (xyzrgb[:,:3]/self._vox).astype(np.int32)
            k = c[:,0]*1_000_000 + c[:,1]*1_000 + c[:,2]
            _,idx = np.unique(k, return_index=True); xyzrgb = xyzrgb[idx]
        hdr = rgb_msg.header; hdr.frame_id = "camera_link"
        self.pub_raw.publish(_make_pc2(hdr, xyzrgb))
        if len(xyzrgb) < 10: return
        gm = ransac_plane(xyzrgb[:,:3], self._ni, self._dt)
        gp = xyzrgb[gm].copy();  gp[:,3:] = [0,200,0]
        op = xyzrgb[~gm].copy(); op[:,3:] = [220,50,50]
        self.pub_gnd.publish(_make_pc2(hdr, gp))
        self.pub_obs.publish(_make_pc2(hdr, op))
        self.get_logger().info(
            f"PC:{len(xyzrgb)} gnd:{gm.sum()} obs:{(~gm).sum()} "
            f"t:{(time.perf_counter()-t0)*1000:.1f}ms")

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
