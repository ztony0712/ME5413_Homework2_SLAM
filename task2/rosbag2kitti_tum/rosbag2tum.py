import rosbag
import transformations
import numpy as np

def write_tum_format_file(output_filename, tum_data):
    with open(output_filename, 'w') as f:
        for line in tum_data:
            f.write(' '.join(map(str, line)) + '\n')

def main(bag_file, output_filename):
    bag = rosbag.Bag(bag_file)
    tum_data = []
    start_time = None

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == 'world' and transform.child_frame_id == 'base_link':
                # Convert quaternion to matrix
                quaternion = [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
                translation = [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]
                mat_world_to_base_link = np.dot(
                    transformations.translation_matrix(translation),
                    transformations.quaternion_matrix(quaternion)
                )
                
                # Extract the translation and quaternion
                trans = transformations.translation_from_matrix(mat_world_to_base_link)
                rot = transformations.quaternion_from_matrix(mat_world_to_base_link)

                # Convert timestamp format
                time = transform.header.stamp.to_sec()
                if start_time is None:
                    start_time = time
                # Reset the timestamp so that it starts at 0
                time -= start_time

                # Add to data list
                tum_data.append([time] + list(trans) + list(rot))

    # Write TUM format data to a file
    write_tum_format_file(output_filename, tum_data)

    bag.close()

if __name__ == '__main__':
    # Read rosbag and write data to TUM format file
    main('vins_gray_fusion.bag', 'vins_gray_fusion_tum123.txt') # rosbag file, output file
