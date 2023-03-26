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

def main():
    InitLIDARCam()
    
    try:
        while True:
            # load camera data, return depth
            frames = pipeline.wait_for_frames()
            depth_data = frames.get_depth_frame()

            # generate depth map
            depth_map = ProcessDepthMap(depth_data)

            # display
            cv2.imshow("L515 Depth Map", depth_map)
            cv2.waitKey(1)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


