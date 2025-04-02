#! /bin/python3

import rospy
import cv2 as cv
import numpy as np
# import time
from event_msgs.msg import Events
import dv_processing as dv
from datetime import timedelta
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


odom_pub = rospy.Publisher('OF/odom', Odometry, queue_size=1)

bridge = CvBridge()
image_pub = rospy.Publisher('Davis_Frame', Image, queue_size=1)

# Initialize global variables
prev_frame = None
prev_flow_x = 0
prev_flow_y = 0
prev_HighestTime = None
prev_seq = None
prev_flow = None

processed_count = 0

visualizer = dv.visualization.EventVisualizer((346,260))

visualizer.setBackgroundColor(dv.visualization.colors.gray())
visualizer.setPositiveColor(dv.visualization.colors.white())
visualizer.setNegativeColor(dv.visualization.colors.black())

count_outliers_x = 0
count_outliers_y = 0

mandatory_combinations = [
    (1, 2),
    (3, 4),
    (5, 6),
    (7, 8),
    (9, 0),
]

dt = 5e-3 # seconds
FOV_y = 0.128  # Meter
FOV_x = 0.108  # Meter
scaleFactor_x = 1 / dt * (FOV_x / 130) 
scaleFactor_y = 1 / dt * (FOV_y / 172) 

def image_callback(msg):
    global prev_frame, prev_flow_x, prev_flow_y, count_outliers_x, count_outliers_y, prev_HighestTime, prev_seq, processed_count, prev_flow   # Declare global variables
    try:
        height = msg.height
        width = msg.width
        
        x = np.array(list(msg.x), dtype=np.uint16)
        y = np.array(list(msg.y), dtype=np.uint16)
        p = np.array(list(msg.polarity), dtype=bool)
        timestamp = np.array(list(msg.timestamp), dtype=np.uint64)
        events = dv.EventStore()
        if len(x) > 0:
            list(map(lambda t: events.push_back(*t), zip(timestamp, x, y, p)))
        
        curr_HighestTime = events.getHighestTime()
        
        noise_filter = dv.noise.BackgroundActivityNoiseFilter(
                    resolution=((width,height)),
                    backgroundActivityDuration=timedelta(microseconds=500)
                )
        
        noise_filter.accept(events)
        events_filtered = noise_filter.generateEvents()
        curr_frame = visualizer.generateImage(events_filtered)
        seq = msg.header.seq
        
        if prev_HighestTime is not None and prev_HighestTime  != 0 and events.size() > 0:
            difference = curr_HighestTime - prev_HighestTime
            if difference > 10_000:
                rospy.logwarn(f'Wrong sequence with time difference: {(difference)}')
                prev_frame = None
                prev_HighestTime = None
                prev_seq = None
                processed_count = 0
                
        if prev_seq is not None:
            if (prev_seq%10, seq%10) not in mandatory_combinations:
                rospy.logwarn(f'Wrong sequence with sequences: {(prev_seq, seq)}')
                prev_frame = None
                prev_HighestTime = None
                prev_seq = None
                processed_count = 0
        
        
        curr_frame = cv.resize(src = curr_frame, dsize = (width  // 2, height // 2), interpolation = cv.INTER_CUBIC)
        curr_frame = cv.cvtColor(curr_frame, cv.COLOR_BGR2GRAY)
        

        if prev_frame is not None:
            # Calculate optical flow
            flow = None
            mean_flow_x = None
            mean_flow_y = None
            if np.abs(prev_flow_x)*scaleFactor_x < 3:
                flow = cv.calcOpticalFlowFarneback(
                prev=prev_frame,
                next=curr_frame,
                flow=prev_flow,
                pyr_scale=0.4,
                levels= 5,
                winsize= 60,
                iterations=5,
                poly_n=4,
                poly_sigma=1.2,
                flags= cv.OPTFLOW_FARNEBACK_GAUSSIAN
                )

            elif np.abs(prev_flow_x)*scaleFactor_x < 5:
                flow = cv.calcOpticalFlowFarneback(
                    prev=prev_frame,
                    next=curr_frame,
                    flow=prev_flow,
                    pyr_scale=0.3,  
                    levels=8,               
                    winsize=100,        
                    iterations=10,           
                    poly_n=6,                
                    poly_sigma=1.5,          
                    flags=cv.OPTFLOW_FARNEBACK_GAUSSIAN  
                )
            else:
                # Downscale frames to reduce motion magnitude
                factor = 1/2
                small_prev = cv.resize(prev_frame, (0, 0), fx=factor, fy=factor)
                small_curr = cv.resize(curr_frame, (0, 0), fx=factor, fy=factor)
                small_prev_flow = cv.resize(prev_flow, (int(width//2 * factor), int(height //2 * factor)))

                # Calculate flow on downscaled frames
                flow_small = cv.calcOpticalFlowFarneback(
                    prev=small_prev,
                    next=small_curr,
                    flow=small_prev_flow,
                    pyr_scale=0.3, 
                    levels=8, 
                    winsize=60, 
                    iterations=8, 
                    poly_n=5, 
                    poly_sigma=1.3, 
                    flags=cv.OPTFLOW_FARNEBACK_GAUSSIAN
                )

                # Upscale flow to original size
                flow = cv.resize(flow_small, (width // 2, height // 2))
                flow[:, :, 0] *= 1/factor * 1.2
                flow[:, :, 1] *= 1/factor * 1.2

            if flow is not None:
                flow_y, flow_x = cv.split(flow)
                mean_flow_x, uncertainty_x = histogram(flow=flow_x, prev_flow_sign=np.sign(prev_flow_x))
                mean_flow_y, uncertainty_y = histogram(flow=flow_y, prev_flow_sign=np.sign(prev_flow_y))
                prev_flow = flow
             
                
            if mean_flow_x is None:
                mean_flow_x = prev_flow_x
                uncertainty_x = 1e3
            if mean_flow_y is None:
                mean_flow_y = prev_flow_y
                uncertainty_y = 1e3
                
            if np.abs(mean_flow_y - prev_flow_y)*scaleFactor_y > 2 and prev_flow_y != 0 and count_outliers_y < 3:
                count_outliers_y += 1
                mean_flow_y = prev_flow_y
            else:
                count_outliers_y = 0
                
            if np.abs(mean_flow_x - prev_flow_x)*scaleFactor_x > 2 and prev_flow_x != 0 and count_outliers_x < 3:
                count_outliers_x += 1
                mean_flow_x = prev_flow_x
            else:
                count_outliers_x = 0

            alpha = 0.6  
            filtered_flow_x = alpha * mean_flow_x + (1 - alpha) * prev_flow_x
            filtered_flow_y = alpha * mean_flow_y + (1 - alpha) * prev_flow_y
            prev_flow_x = filtered_flow_x
            prev_flow_y = filtered_flow_y

            # Publish the filtered velocities
            odom = Odometry()
            odom.twist.twist.linear.x = np.round(-filtered_flow_x * scaleFactor_x, 2)
            odom.twist.twist.linear.y = np.round(filtered_flow_y * scaleFactor_y,2)
            
            variance_x = uncertainty_x ** 2
            variance_y = uncertainty_y ** 2
            
            # Covariance for the velocities (6x6)
            # Fill in only the variances for x (index 0) and y (index 7)
            odom.twist.covariance = [0] * 36  # 6x6 matrix initialized to zeros
            odom.twist.covariance[0] = variance_x  # Variance for twist.twist.linear.x
            odom.twist.covariance[7] = variance_y  # Variance for twist.twist.linear.y

            # Publish the message
            odom_pub.publish(odom)
            odom_pub.publish(odom)
        
        # Publish the current frame as an image
        image_pub.publish(bridge.cv2_to_imgmsg(curr_frame, encoding="mono8"))
            
        if prev_frame is None:
            prev_frame = curr_frame
            prev_HighestTime = curr_HighestTime
            prev_seq = seq
        else:
            prev_frame = None
            prev_HighestTime = None
            prev_seq = None

    except Exception as e:
        rospy.logerr("Error processing events: %s", e)



def histogram(flow, prev_flow_sign, hist_size=50):
    # Check if the flow array is empty
    if flow.size == 0:
        return 0, 0  # Return 0 for both value and uncertainty

    # Flatten the flow array to create a 1D array
    flow = flow.flatten()

    # Check if the flow array has a valid range
    flow_min, flow_max = flow.min(), flow.max()
    if flow_min == flow_max:
        return flow_min, 0  # Return 0 for both value and uncertainty if all values are the same

    # Calculate interquartile range (IQR)
    Q1 = np.percentile(flow, 35)
    Q3 = np.percentile(flow, 65)
    IQR = Q3 - Q1

    # Define thresholds using a multiplier k
    k = 1.5 
    lower_threshold = Q1 - k * IQR
    upper_threshold = Q3 + k * IQR

    # Calculate indices for the threshold range
    lower_index = int((lower_threshold - flow_min) / (flow_max - flow_min) * hist_size)
    upper_index = int((upper_threshold - flow_min) / (flow_max - flow_min) * hist_size)

    # Check if the distance between the thresholds is too far
    if upper_index - lower_index > hist_size * 0.5:
        # Maximum number of iterations
        max_iterations = 10
        iteration = 0

        while upper_index - lower_index > hist_size * 0.5:
            k *= 0.9  # Reduce sensitivity iteratively
            lower_threshold = Q1 - k * IQR
            upper_threshold = Q3 + k * IQR
            lower_index = int((lower_threshold - flow_min) / (flow_max - flow_min) * hist_size)
            upper_index = int((upper_threshold - flow_min) / (flow_max - flow_min) * hist_size)
            
            # Increment iteration count
            iteration += 1
            
            # Break loop if max iterations are reached
            if iteration >= max_iterations:
                value = np.min(flow) if prev_flow_sign == -1 else np.max(flow)
                return value, IQR  # Return the flow value and IQR as uncertainty

        # Compute valid values and return the mean
        valid_values = flow[(flow >= lower_threshold) & (flow <= upper_threshold)]
        if valid_values.size > 0:
            mean_value = valid_values.mean()
            uncertainty = np.std(valid_values)  # Uncertainty as standard deviation
            return mean_value, uncertainty  # Return mean and uncertainty
        else:
            return 0, 1e3  # If no valid values, return 0 for both

    # Calculate the mean of values within the threshold range
    valid_values = flow[(flow >= lower_threshold) & (flow <= upper_threshold)]
    if valid_values.size > 0:
        mean_value = valid_values.mean()
        uncertainty = np.std(valid_values)  # Uncertainty as standard deviation
    else:
        mean_value = 0
        uncertainty = 1e3
    return mean_value, uncertainty

def main():
    rospy.init_node('live_frame_reader_bag', anonymous=True)
    rospy.Subscriber('/events', Events, image_callback)
    rospy.loginfo("Subscribed to 'events' topic.")
    rospy.spin()


if __name__ == '__main__':
    main()
