"""Stage 3 V2: light RGB-confirmed one-shot obstacle tree semantic markers.

Uses existing Stage 2 world LiDAR clouds, camera images, camera_info and base
odometry. Does not alter prior perception, cutting/control, or any on-disk map.
Registered trees never get fitted or updated again in this ROS process.
"""
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

from .tree_vision_geometry import (VisionTreeConfig, select_rgb_confirmed_points,
                                   cluster_canopy_observations, refine_tree_base_xy)

MODEL_FILE = (Path.home() / 'UAV_lumberjack/sim/rviz_assets/'
              'forest_tree_broadleaf_04/forest_tree_broadleaf_04.dae')
# User-converted GLB->DAE mesh geometric height; used for ONE uniform scale.
MODEL_HEIGHT = 14.62181


def get_stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def xyz_cloud(msg):
    if not msg.data or msg.width * msg.height == 0:
        return np.empty((0,3),np.float32)
    fs = {f.name: f for f in msg.fields}
    if any(k not in fs or fs[k].datatype != PointField.FLOAT32 for k in ('x','y','z')):
        return np.empty((0,3),np.float32)
    endian = '>' if msg.is_bigendian else '<'
    dt = np.dtype({'names':['x','y','z'], 'formats':[endian+'f4']*3,
                   'offsets':[fs[k].offset for k in ('x','y','z')],
                   'itemsize':msg.point_step})
    dat = np.ndarray((msg.height,msg.width),dtype=dt,
                     buffer=memoryview(msg.data),
                     strides=(msg.row_step,msg.point_step))
    xyz = np.column_stack([dat[k].reshape(-1) for k in ('x','y','z')])
    return xyz[np.isfinite(xyz).all(axis=1)].astype(np.float32,copy=False)


def image_bgr(msg):
    channels = 4 if msg.encoding in ('rgba8','bgra8') else 3
    if msg.encoding not in ('rgb8','bgr8','rgba8','bgra8'):
        raise ValueError('Unsupported Image encoding: '+msg.encoding)
    row_bytes=msg.width*channels
    data=np.frombuffer(msg.data,np.uint8)
    if len(data) < msg.height*msg.step or msg.step < row_bytes:
        raise ValueError('Invalid Image step/data')
    buf=data[:msg.height*msg.step].reshape(msg.height,msg.step)[:,:row_bytes]
    frame=buf.reshape(msg.height,msg.width,channels)
    code={'rgb8':cv2.COLOR_RGB2BGR,'bgr8':None,
          'rgba8':cv2.COLOR_RGBA2BGR,'bgra8':cv2.COLOR_BGRA2BGR}[msg.encoding]
    if code is not None:frame=cv2.cvtColor(frame,code)
    if msg.width>960:
        frame=cv2.resize(frame,(960,round(msg.height*960/msg.width)),
                         interpolation=cv2.INTER_AREA)
    return np.ascontiguousarray(frame)


class TreeSemanticMapper(Node):
    def __init__(self):
        super().__init__('tree_semantic_mapper')
        self.declare_parameter('nonground_topic','/perception/lidar/points_nonground')
        self.declare_parameter('ground_topic','/perception/lidar/points_ground')
        self.declare_parameter('image_topic','/camera/image_raw')
        self.declare_parameter('camera_info_topic','/camera/camera_info')
        self.declare_parameter('odom_topic','/model/x500_lumberjack/base_link_odometry')
        self.declare_parameter('target_topic','/perception/target_branch_cloud_world')
        self.declare_parameter('window_sec',8.0)
        self.declare_parameter('detect_hz',1.0)
        self.declare_parameter('min_distinct_images',4)
        self.declare_parameter('min_evidence_span_sec',2.5)
        self.declare_parameter('min_pending_sec',2.0)
        self.declare_parameter('max_pending_xy_drift_m',0.75)
        self.declare_parameter('min_unique_voxels',14)
        self.declare_parameter('range_m',26.0)
        self.declare_parameter('max_sensor_time_delta_sec',.35)
        self.declare_parameter('min_local_texture_std',8.0)
        self.declare_parameter('show_candidates',True)
        self.declare_parameter('mesh_file',str(MODEL_FILE))
        self.declare_parameter('marker_topic','/perception/obstacle_trees/markers')
        self.declare_parameter('candidate_topic','/perception/obstacle_trees/candidates')
        def p(k):return self.get_parameter(k).value
        self.cfg=VisionTreeConfig(search_range_m=float(p('range_m')),
                    min_distinct_images=int(p('min_distinct_images')),
                    min_evidence_span_sec=float(p('min_evidence_span_sec')),
                    min_unique_voxels=int(p('min_unique_voxels')),
                    min_local_texture_std=float(p('min_local_texture_std')))
        self.window_sec=float(p('window_sec'))
        self.min_pending_sec=float(p('min_pending_sec'))
        self.max_pending_drift=float(p('max_pending_xy_drift_m'))
        self.max_delta=float(p('max_sensor_time_delta_sec'))
        self.show_candidates=bool(p('show_candidates'))
        self.mesh=Path(str(p('mesh_file')))
        if not self.mesh.is_file():
            raise RuntimeError('Tree DAE missing: '+str(self.mesh))
        self.odoms=deque(maxlen=200)
        self.clouds=deque(maxlen=50)
        self.grounds=deque(maxlen=50)
        self.foliage=deque(maxlen=18)
        self.camera_k=None
        self.latest_image=None
        self.last_image_stamp=None
        self.last_log_stamp=-1.
        self.confirmed=[]
        self.pending=[]   # short-lived verification; never persist to disk
        self.next_id=1
        self.target_xy=None
        self.target_time=-1.
        qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_tree=self.create_publisher(MarkerArray,str(p('marker_topic')),qos)
        self.pub_candidates=self.create_publisher(MarkerArray,str(p('candidate_topic')),10)
        # Clear permanent V1 RViz markers when switching algorithms.
        clear=Marker()
        clear.header.frame_id='world'
        clear.action=Marker.DELETEALL
        self.pub_tree.publish(MarkerArray(markers=[clear]))
        self.pub_candidates.publish(MarkerArray(markers=[clear]))
        self.create_subscription(PointCloud2,str(p('nonground_topic')),self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,str(p('ground_topic')),self.on_ground,qos_profile_sensor_data)
        self.create_subscription(Image,str(p('image_topic')),self.on_image,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,str(p('camera_info_topic')),self.on_info,qos_profile_sensor_data)
        self.create_subscription(Odometry,str(p('odom_topic')),self.on_odom,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,str(p('target_topic')),self.on_target,qos_profile_sensor_data)
        self.create_timer(1.0/max(.5,float(p('detect_hz'))),self.process_image)
        # Mesh data are cached in RViz; occasional snapshots also serve late RViz subscribers.
        self.create_timer(20.0,self.publish_registered)
        self.get_logger().info('[TREE MAP]\n'
            '  status       : ready (V2.1 sealed RGB + LiDAR)\n'
            '  input        : /perception/lidar/points_nonground + /camera/image_raw\n'
            '  mapping      : one-shot world static\n'
            '  confirmation : %d RGB views / >=%.1fs span + stable center\n'
            '  model        : forest_tree_broadleaf_04 (UNIFORM scale)\n'
            '  markers      : %s' % (self.cfg.min_distinct_images,
                 self.cfg.min_evidence_span_sec,str(p('marker_topic'))))

    def sim_now(self):return self.get_clock().now().nanoseconds*1e-9

    def trim(self):
        cutoff=self.sim_now()-max(self.window_sec,3.)
        for q in (self.clouds,self.grounds,self.foliage):
            while q and q[0][0] < cutoff:q.popleft()

    def on_odom(self,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        values=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w],np.float64)
        if not np.isfinite(values).all() or np.linalg.norm(values[3:])<1e-7:return
        stamp=get_stamp(msg)
        if self.odoms and stamp<self.odoms[-1][0]:self.odoms.clear()
        self.odoms.append((stamp,values[:3],values[3:]))

    def on_cloud(self,msg):
        if msg.header.frame_id!='world':return
        self.clouds.append((get_stamp(msg),xyz_cloud(msg)))
        self.trim()

    def on_ground(self,msg):
        if msg.header.frame_id!='world':return
        self.grounds.append((get_stamp(msg),xyz_cloud(msg)))
        self.trim()

    def on_image(self,msg):
        self.latest_image=msg  # one frame only, not an RGB history

    def on_info(self,msg):
        if msg.width and msg.height and msg.k[0]>0 and msg.k[4]>0:
            self.camera_k=(float(msg.k[0]),float(msg.k[4]),float(msg.k[2]),
                           float(msg.k[5]),int(msg.width),int(msg.height))

    def on_target(self,msg):
        if msg.header.frame_id!='world':return
        pts=xyz_cloud(msg)
        if len(pts)>=5:
            self.target_xy=np.median(pts[:,:2],axis=0)
            self.target_time=self.sim_now()

    def image_pose(self,stamp):
        if not self.odoms:return None
        od=list(self.odoms)
        times=np.array([x[0] for x in od])
        i=int(np.searchsorted(times,stamp))
        if i==0 or i==len(od):
            near=od[0] if i==0 else od[-1]
            if abs(stamp-near[0])>.25:return None
            return near[1],Rotation.from_quat(near[2]).as_matrix()
        before,after=od[i-1],od[i]
        if after[0]-before[0]>1.0:return None
        ratio=(stamp-before[0])/(after[0]-before[0])
        xyz=before[1]*(1-ratio)+after[1]*ratio
        R=Slerp([before[0],after[0]],Rotation.from_quat([before[2],after[2]]))([stamp]).as_matrix()[0]
        return xyz,R

    def process_image(self):
        msg=self.latest_image
        if msg is None or self.camera_k is None or not self.clouds or not self.grounds:
            self.get_logger().warn('[TREE MAP] waiting for RGB, CameraInfo, ground and nonground',
                                   throttle_duration_sec=15.)
            return
        stamp=get_stamp(msg)
        if stamp==self.last_image_stamp:return
        self.last_image_stamp=stamp
        best=min(self.clouds,key=lambda x:abs(x[0]-stamp))
        gbest=min(self.grounds,key=lambda x:abs(x[0]-best[0]))
        if abs(best[0]-stamp)>self.max_delta or abs(gbest[0]-best[0])>self.max_delta:
            return
        pose=self.image_pose(stamp)
        if pose is None:return
        try:
            frame=image_bgr(msg)
            exclusions=[(rec['xy'],max(self.cfg.exclude_radius_m,.45*rec['crown_d']))
                        for rec in self.confirmed]
            if self.target_xy is not None and self.sim_now()-self.target_time<3.:
                exclusions.append((self.target_xy,2.5))
            valid=select_rgb_confirmed_points(best[1],frame,self.camera_k,
                                              pose[0],pose[1],gbest[1],self.cfg,
                                              exclude_xy=exclusions)
            self.foliage.append((stamp,valid))
            self.trim()
            ground=np.concatenate([g[1] for g in list(self.grounds)[-8:]])
            target=self.target_xy if self.sim_now()-self.target_time<3. else None
            obs=cluster_canopy_observations(self.foliage,ground,self.cfg,
                                            mapped=self.confirmed,target_xy=target)
            if self.show_candidates:self.publish_candidates(obs)
            # Candidate must remain spatially stable across fresh processing
            # cycles, in addition to having >=4 separate RGB observations.
            self.pending = [q for q in self.pending if stamp-q['seen'] < 12.0]
            for candidate in obs:
                xy = np.asarray(candidate['xy'], dtype=np.float64)
                if any(np.linalg.norm(xy-np.asarray(t['xy'])) <
                       max(self.cfg.exclude_radius_m,.45*t['crown_d'])
                       for t in self.confirmed):
                    continue
                match = None
                for q in self.pending:
                    if np.linalg.norm(xy-q['xy']) < 1.25:
                        match=q
                        break
                if match is None:
                    self.pending.append(dict(xy=xy.copy(), first=stamp,
                                             seen=stamp, checks=1))
                    continue
                drift = np.linalg.norm(xy-match['xy'])
                if drift > self.max_pending_drift:
                    match.update(xy=xy.copy(), first=stamp, seen=stamp, checks=1)
                    continue
                match['seen']=stamp
                match['checks']+=1
                if stamp-match['first'] < self.min_pending_sec or match['checks'] < 3:
                    continue
                tree=dict(candidate)
                # LiDAR trunk anchoring is OPTIONAL. Do not distort the model
                # or reject trees whose lower trunk is occluded.
                anchor, origin = refine_tree_base_xy(
                    tree['xy'],tree['ground_z'],tree['height'],tree['crown_d'],
                    list(self.clouds)[-12:])
                tree['xy']=anchor.tolist()
                tree['anchor']=origin
                tree['id']=self.next_id
                self.next_id+=1
                self.confirmed.append(tree)
                self.pending = [q for q in self.pending if q is not match]
                self.get_logger().info('[TREE MAP] registered #%d x=%.2f y=%.2f '
                    'height≈%.1fm views=%d anchor=%s | total=%d' %
                    (tree['id'],tree['xy'][0],tree['xy'][1],tree['height'],
                     tree['frames'],origin,len(self.confirmed)))
                self.publish_registered()
            if self.sim_now()-self.last_log_stamp>20.:
                self.last_log_stamp=self.sim_now()
                self.get_logger().info('[TREE MAP] confirmed=%d pending_foliage=%d '
                    'recent_rgb_views=%d' %
                    (len(self.confirmed),sum(len(q[1]) for q in self.foliage),len(self.foliage)))
        except Exception as exc:
            self.get_logger().error('[TREE MAP] V2.1 process failed: '+str(exc),
                                    throttle_duration_sec=5.0)

    def marker(self,ns,uid):
        m=Marker()
        m.header.frame_id='world'
        m.header.stamp=self.get_clock().now().to_msg()
        m.ns=ns;m.id=int(uid);m.action=Marker.ADD
        m.pose.orientation.w=1.0
        m.lifetime=Duration(sec=0,nanosec=0)
        return m

    def publish_registered(self):
        msg=MarkerArray()
        for tree in self.confirmed:
            marker=self.marker('obstacle_tree_mesh',tree['id'])
            marker.type=Marker.MESH_RESOURCE
            marker.mesh_resource=self.mesh.as_uri()
            marker.mesh_use_embedded_materials=True
            marker.pose.position.x=float(tree['xy'][0]);marker.pose.position.y=float(tree['xy'][1])
            marker.pose.position.z=float(tree['ground_z'])
            scale=float(np.clip(tree['height']/MODEL_HEIGHT,.15,.56))
            # User requirement: original mesh proportions are NEVER stretched.
            marker.scale.x=marker.scale.y=marker.scale.z=scale
            marker.color.r=marker.color.g=marker.color.b=marker.color.a=1.0
            msg.markers.append(marker)
        if msg.markers:self.pub_tree.publish(msg)

    def publish_candidates(self,obs):
        arr=MarkerArray()
        for i,t in enumerate(obs):
            m=self.marker('tree_candidates',i);m.type=Marker.SPHERE
            m.pose.position.x=float(t['xy'][0]);m.pose.position.y=float(t['xy'][1]);m.pose.position.z=float(t['ground_z']+t['height']*.6)
            m.scale.x=m.scale.y=m.scale.z=.3
            m.color.r=1.;m.color.g=.8;m.color.b=.05;m.color.a=.7
            m.lifetime=Duration(sec=2,nanosec=0)
            arr.markers.append(m)
        if arr.markers:self.pub_candidates.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node=TreeSemanticMapper()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
