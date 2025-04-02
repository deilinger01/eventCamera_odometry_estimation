#! /bin/python3

import rospy
import dv_processing as dv
import cv2 as cv
import numpy as np
import time
from datetime import timedelta
from event_msgs.msg import Events

count = 0
COUNT_RESET = 20 # Accumulated events are published ate every COUNT_RESET and COUNT_RESET + 1 iteration

def process_accumulated_events():
    """Initialize the ROS node, configure the event camera, and process event streams."""
    rospy.init_node('dvs_camera_node')
    publisher = rospy.Publisher('events', Events, queue_size=2)
    
    # Initialize the event camera capture
    capture = dv.io.CameraCapture()
    
    # Path to camera calibration file
    calibration_file_path = "/home/racecar/catkin_ws/src/f1tenth_system/odometry_estimation/DAVIS-00826001.xml"
    
    # Load camera calibration settings
    calibration_set = dv.camera.CalibrationSet.LoadFromFile(calibration_file_path)
    
    for designation, calibration in calibration_set.getCameraCalibrations().items():
        rospy.loginfo(f"[{designation}] Found calibration for camera with name [{calibration.name}]")
        geometry = dv.camera.CameraGeometry(
            distortion=calibration.distortion,
            focal_length_x=calibration.focalLength[0],
            focal_length_y=calibration.focalLength[1],
            principal_point_x=calibration.principalPoint[0],
            principal_point_y=calibration.principalPoint[1],
            resolution=capture.getEventResolution(),
            distortion_model=calibration.distortionModel
        )
    
    # Ensure the input camera provides an event stream
    if not capture.isEventStreamAvailable():
        raise RuntimeError("Input camera does not provide an event stream.")
    
    # Initialize a visualizer for the event stream
    visualizer = dv.visualization.EventVisualizer(capture.getEventResolution())
    
    # Configure the color scheme for the visualizer
    visualizer.setBackgroundColor(dv.visualization.colors.gray())
    visualizer.setPositiveColor(dv.visualization.colors.white())
    visualizer.setNegativeColor(dv.visualization.colors.black())
    
    # Initialize a slicer for the event stream
    event_slicer = dv.EventStreamSlicer()
    
    def slicing_callback(event_store: dv.EventStore):
        """Callback function to process a batch of events and publish them as a ROS message."""
        global count
        if count < 2:
            events_array = event_store.numpy()

            # Extract the event fields from the numpy array
            x_positions = events_array['x'].astype(np.uint16).tolist()
            y_positions = events_array['y'].astype(np.uint16).tolist()
            polarities = events_array['polarity'].astype(np.uint8).tolist()
            timestamps = events_array['timestamp'].astype(np.uint64).tolist()

            # Create and populate the ROS Events message
            event_message = Events()
            event_message.header.stamp = rospy.Time.now()
            event_message.header.frame_id = "EventBatch"
            event_message.height = capture.getEventResolution()[1]
            event_message.width = capture.getEventResolution()[0]
            event_message.x = x_positions
            event_message.y = y_positions
            event_message.polarity = polarities
            event_message.timestamp = timestamps

            # Publish the message
            publisher.publish(event_message)
            
        count = count + 1 if count < COUNT_RESET else 0

    event_slicer.doEveryTimeInterval(timedelta(milliseconds=5), slicing_callback)

    # Continuously process event batches while the camera is running
    while capture.isRunning():
        # Receive the next batch of events
        event_batch = capture.getNextEventBatch()
        
        # Check if any events were received
        if event_batch is not None:
            
            # Undistort the events using the camera calibration
            undistorted_events = geometry.undistortEvents(event_batch)
            
            # Pass the undistorted events to the event slicer
            if undistorted_events is not None:
                event_slicer.accept(undistorted_events)

def main():
    """Main entry point of the script."""
    process_accumulated_events()

if __name__ == '__main__':
    main()
