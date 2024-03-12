import rosbag
import numpy as np
import transformations

def write_kitti_format_file(output_filename, kitti_data):
    with open(output_filename, 'w') as f:
        for transform_matrix in kitti_data:
            # Only take the 3x4 part of the transformation matrix
            kitti_matrix = transform_matrix[:3, :]
            # Convert the matrix to a string format, space-separated
            kitti_format = ' '.join(map('{:.12e}'.format, kitti_matrix.flatten()))
            f.write(kitti_format + '\n')

def main(bag_file, output_filename):
    bag = rosbag.Bag(bag_file)
    kitti_data = []

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == 'world' and transform.child_frame_id == 'base_link':
                # Extract the quaternion
                quaternion = np.array([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])
                # Extract the translation
                translation = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                # Construct the 4x4 transformation matrix
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = transformations.quaternion_matrix(quaternion)[:3, :3]
                transform_matrix[:3, 3] = translation

                kitti_data.append(transform_matrix)

    # Write KITTI format data to file
    write_kitti_format_file(output_filename, kitti_data)

    bag.close()

# Corrected import statement for transformations module
from tf.transformations import quaternion_matrix

if __name__ == '__main__':
    main('vins_gray_fusion.bag', 'vins_gray_fusion_kitti.txt')
