#!/usr/bin/env python3
"""Capture camera images and Velodyne point clouds for Prius calibration."""
import argparse, threading, time
from pathlib import Path
import cv2, numpy as np, yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

DEFAULT_CAMERA_TOPICS = {
    'camera_front': '/camera/device_0/image_raw',
    'camera_front_left': '/camera/device_2/image_raw',
    'camera_front_right': '/camera/device_4/image_raw',
    'camera_back': '/camera/device_6/image_raw',
    'camera_back_left': '/camera/device_8/image_raw',
    'camera_back_right': '/camera/device_10/image_raw',
}

def image_msg_to_cv2(msg):
    h,w = msg.height,msg.width; enc = msg.encoding.lower(); data=np.frombuffer(msg.data,dtype=np.uint8)
    if enc in ('bgr8','8uc3'): return data.reshape((h,w,3)).copy()
    if enc == 'rgb8': return cv2.cvtColor(data.reshape((h,w,3)), cv2.COLOR_RGB2BGR)
    if enc in ('mono8','8uc1'): return data.reshape((h,w)).copy()
    if enc in ('uyvy','yuv422','uyvy422'):
        return cv2.cvtColor(data.reshape((h,w,2)), cv2.COLOR_YUV2BGR_UYVY)
    raise RuntimeError(f'Unsupported image encoding: {msg.encoding}')

def write_ascii_pcd(path, cloud):
    fields = [f.name for f in cloud.fields]
    wanted = ['x','y','z'] + (['intensity'] if 'intensity' in fields else [])
    pts = list(point_cloud2.read_points(cloud, field_names=wanted, skip_nans=True))
    with open(path,'w',encoding='utf-8') as f:
        f.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
        if 'intensity' in wanted:
            f.write('FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
        else:
            f.write('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA ascii\n')
        for p in pts:
            f.write(' '.join(f'{float(v):.6f}' for v in p)+'\n')
    return len(pts)

class CaptureNode(Node):
    def __init__(self, cams, lidar):
        super().__init__('prius_calibration_capture')
        self.lock=threading.Lock(); self.imgs={}; self.cloud=None; self.subs=[]
        for name,topic in cams.items():
            self.get_logger().info(f'Sub image {name}: {topic}')
            self.subs.append(self.create_subscription(Image, topic, lambda m,n=name:self.on_img(n,m), 10))
        self.get_logger().info(f'Sub lidar: {lidar}')
        self.subs.append(self.create_subscription(PointCloud2, lidar, self.on_cloud, 10))
    def on_img(self,n,m):
        with self.lock: self.imgs[n]=m
    def on_cloud(self,m):
        with self.lock: self.cloud=m
    def snapshot(self):
        with self.lock: return dict(self.imgs), self.cloud

def load_topics(path):
    if not path: return DEFAULT_CAMERA_TOPICS, '/velodyne_points'
    cfg=yaml.safe_load(open(path,'r',encoding='utf-8'))
    cams={k:v['image_topic'] for k,v in cfg.get('cameras',{}).items()}
    lidar=cfg.get('lidar',{}).get('pointcloud_topic','/velodyne_points')
    return cams,lidar

def stamp_s(st): return float(st.sec)+float(st.nanosec)*1e-9

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',default='/colcon_ws/calibration_data')
    ap.add_argument('--topics',default=''); ap.add_argument('--wait-sec',type=float,default=2.0)
    args=ap.parse_args(); cams,lidar=load_topics(args.topics); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=CaptureNode(cams,lidar); threading.Thread(target=rclpy.spin,args=(node,),daemon=True).start()
    print(f'Output: {out}\nPress Enter to capture pose, q Enter to quit.')
    i=0
    try:
        while rclpy.ok():
            s=input(f'[pose {i:03d}] > ').strip().lower()
            if s in ('q','quit','exit'): break
            time.sleep(args.wait_sec); imgs,cloud=node.snapshot(); d=out/f'pose_{i:03d}'; d.mkdir(parents=True,exist_ok=True)
            meta=[f'capture_unix_time: {time.time():.6f}']
            saved=0
            for name in cams:
                msg=imgs.get(name)
                if msg is None: meta.append(f'{name}: MISSING'); continue
                img=image_msg_to_cv2(msg); cv2.imwrite(str(d/f'{name}.png'), img); saved+=1
                meta.append(f'{name}: stamp={stamp_s(msg.header.stamp):.9f}, encoding={msg.encoding}, size={msg.width}x{msg.height}')
            if cloud is not None:
                n=write_ascii_pcd(d/'velodyne.pcd', cloud); meta.append(f'velodyne: stamp={stamp_s(cloud.header.stamp):.9f}, points={n}')
            else: meta.append('velodyne: MISSING')
            (d/'meta.txt').write_text('\n'.join(meta)+'\n',encoding='utf-8')
            print(f'saved {saved}/{len(cams)} cameras + lidar to {d}')
            i+=1
    finally:
        node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
