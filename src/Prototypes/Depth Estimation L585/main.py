import cv2
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()

def InitLIDARCam():
    # L515 config menu
    config = rs.config()

    # enable stream of depth data from cam
    config.enable_stream(rs.stream.depth)

    # start pipeline
    pipeline.start(config)

def ProcessDepthMap(depth_data):
    # convert to depth data to numpy array
    depth_image = np.asanyarray(depth_data.get_data())

    # apply depth colormap design on the depth image
    return cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), colormap=cv2.COLORMAP_JET)

def CreatePointcloud3D(depth_data):
    # Init PointCloud Generator instance
    pc = rs.pointcloud()
    
    # calculate Pointcloud points and generate cloud
    pc_points = pc.calculate(depth_data)
    pointcloud = np.asanyarray(pc_points.get_vertices()).view(np.float32).reshape(-1, 3) 

    return pointcloud

def main():
    try:
        while True:
            # load camera data, return depth
            frames = pipeline.wait_for_frames()
            depth_data = frames.get_depth_frame()

            # generate depth map
            depth_map = ProcessDepthMap(depth_data)

            # generate Pointcloud
            pointcloud = CreatePointcloud3D(depth_data)

            # display
            cv2.imshow("L515 Depth Map", depth_map)
            cv2.imshow("L515 Depth Pointcloud", pointcloud)
            
            if cv2.waitKey(20) & 0xff == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
