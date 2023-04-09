import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2

pipeline = rs.pipeline()

CAM_HEIGHT = 2.5

def InitLIDARCam():
    # L515 config menu 
    config = rs.config()

    # enable depth stream
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)

    # start streaming
    pipeline.start(config)

def ProcessDepthMap(frames):
    # get depth infos
    depth_frame = frames.get_depth_frame()

    # create depth image as numpy array and depth map display
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), colormap=cv2.COLORMAP_JET)

    # return depth image and map
    return depth_image, depth_map

def CreateBirdseye3D(depth_image):
    # create visualization window with Pointcloud display 
    vis = o3d.visualization.ExternalVisualizer()
    vis.create_window()
    vis.add_geometry(o3d.geometry.PointCloud())

    # get Pointcloud using intrinsic camera data
    intrinsics = o3d.camera.PinholeCameraIntrinsic(width=1024, height=768, fx=612.9648, fy=612.9648, cx=512, cy=384)
    pcd = o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_image), intrinsics)

    # pointcloud array in [X Y Z]-Coordinates
    xyz_array = np.asarray(pcd.points)

    # init birdseyeview placeholder
    birdseyeview = np.zeros((int(intrinsics.height), int(intrinsics.width)))
    birdseyeview.fill(np.nan)

    # generate birdseyeview from depth by setting the z-axis to the cam height
    x, y = np.round(intrinsics.project(xyz_array[:, :3])).astype(int).T
    valid = (x >= 0) & (x < birdseyeview.shape[1]) & (y >= 0) & (y < birdseyeview.shape[0])
    birdseyeview[x[valid], y[valid]] = (xyz_array[:, 2][valid]) - CAM_HEIGHT

    # init view
    birdseyeview_pc = o3d.geometry.PointCloud()

    # init points in pointcloud 
    birdseyeview_pc.points = o3d.utility.Vector3dVector(
        np.transpose([np.tile(np.arange(0, birdseyeview.shape[1]), birdseyeview.shape[0]), 
                      np.repeat(np.arange(0, birdseyeview.shape[0]), birdseyeview.shape[1]), birdseyeview.flatten()])
    )
    
    birdseyeview_pc.paint_uniform_color([0.5, 0.5, 0.5])

    # display
    vis.update_geometry(birdseyeview_pc)
    vis.poll_events()
    vis.update_renderer()


def main():
    while True:
        frames = pipeline.wait_for_frames()

        # Display Depth Map
        depth_image, depth_map = ProcessDepthMap(frames)
        cv2.imshow("Depth Map L515", depth_map)

        CreateBirdseye3D(depth_image)

        if cv2.waitKey(20) & 0xff == ord("q"):
            break

    pipeline.stop()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()

