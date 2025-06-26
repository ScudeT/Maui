import os
import sys
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message, serialize_message

# ---- SETUP PARAMETERS HERE ----
IMU_COV_ORI = 0.001        # Value for orientation covariance diagonal
IMU_COV_ANGVEL = 0.01      # Value for angular velocity covariance diagonal
IMU_COV_LINACC = 1.0       # Value for linear acceleration covariance diagonal
POSE_Z_COV = 0.001         # Value for pose.z covariance

INPUT_BAG = os.path.abspath("rosbag2_2025_05_23-18_17_54")    # change as needed
OUTPUT_BAG = os.path.abspath("test")  # change as needed
# --------------------------------

imu_msg_type = 'sensor_msgs/msg/Imu'
pose_msg_type = 'geometry_msgs/msg/PoseWithCovarianceStamped'

def main():
    # Setup reader
    reader = SequentialReader()
    storage_options = StorageOptions(uri=INPUT_BAG, storage_id='mcap')
    reader.open(storage_options, ConverterOptions('', ''))



    # Prepare writer
    if os.path.exists(OUTPUT_BAG):
        print(f"Output bag {OUTPUT_BAG} exists, remove it or choose another name.")
        sys.exit(1)
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=OUTPUT_BAG, storage_id='mcap'), ConverterOptions('', ''))
    # Copy topic info
    topic_types = {}
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)
        topic_types[topic.name] = topic.type

    # Message classes (lazy-load for speed)
    from rosidl_runtime_py.utilities import get_message

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        type_ = topic_types[topic]

        # IMU case
        if type_ == imu_msg_type:
            msg_class = get_message(imu_msg_type)
            msg = deserialize_message(data, msg_class)
            # Set orientation covariance
            msg.orientation_covariance = [0.0]*9
            for i in [0, 4, 8]:
                msg.orientation_covariance[i] = IMU_COV_ORI
            # Set angular velocity covariance
            msg.angular_velocity_covariance = [0.0]*9
            for i in [0, 4, 8]:
                msg.angular_velocity_covariance[i] = IMU_COV_ANGVEL
            # Set linear acceleration covariance
            msg.linear_acceleration_covariance = [0.0]*9
            for i in [0, 4, 8]:
                msg.linear_acceleration_covariance[i] = IMU_COV_LINACC
            new_data = serialize_message(msg)
            writer.write(topic, new_data, t)

        # PoseWithCovarianceStamped case
        elif type_ == pose_msg_type:
            msg_class = get_message(pose_msg_type)
            msg = deserialize_message(data, msg_class)
            # All covariances to zero, except pose.z (index 14)
            msg.pose.covariance = [0.0]*36
            msg.pose.covariance[14] = POSE_Z_COV
                   
            msg.pose.covariance[0 * 6 + 0] = 0.1
            msg.pose.covariance[1 * 6 + 1] = 0.1
            msg.pose.covariance[3 * 6 + 3] = 1e6
            msg.pose.covariance[4 * 6 + 4] = 1e6
            msg.pose.covariance[5 * 6 + 5] = 1e6

            new_data = serialize_message(msg)
            writer.write(topic, new_data, t)

        # Other messages: copy as-is
        else:
            writer.write(topic, data, t)

    print("Bag processing complete. Output written to:", OUTPUT_BAG)

if __name__ == '__main__':
    main()
