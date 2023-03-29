# This code can be used to plot the trajectory of the L515 LIDAR Camera in 3D Space
# Applications: Flight trajectory in 3D, autonomous driving history

import cv2
import pyrealsense2 as rs
import numpy as np
import matplotlib.pyplot as plt

pipeline = rs.pipeline()

def InitLIDARCam():
    # L515 config menu
    config = rs.config()

    # enable data stream of IMU data
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    # start stream pipeline
    pipeline.start(config)

def PlotTrajectory3D(gyro_frame):
    # setup 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection="3D")

    # placeholder for trajectory line in 3D space (array([x, y, z]))
    trajectory3D = np.array([0, 0, 0])

    # if data frame is not NULL
    if gyro_frame:
        # process gyro data into [X Y Z]-format
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        gyro_values = np.array([gyro_data.x, gyro_data.y, gyro_data.z])

        # update trajectory line
        trajectory3D += gyro_values
        
        # plot the trajectory of the camera in 3D space
        ax.scatter(trajectory3D[0], trajectory3D[1], trajectory3D[2], c="b")
        plt.draw()

        # delay
        plt.pause(0.0001)
        

def main():
    while True:
        # get IMU-data from camera
        frame = pipeline.wait_for_frames()
        gyro_frame = frame.first_or_default(rs.stream.gyro)

        # plot camera movement
        PlotTrajectory3D(gyro_frame)

        # quit
        if cv2.waitKey(20) & 0xff == ord("q"):
            break

    pipeline.stop()

if __name__ == "__main__":
    main()

