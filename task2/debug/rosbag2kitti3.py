import rosbag
import numpy as np
import transformations

def get_static_transform(bag_file, source_frame, target_frame):
    with rosbag.Bag(bag_file, 'r') as bag:
        for _, msg, _ in bag.read_messages(topics=['/tf_static']):
            for transform in msg.transforms:
                if transform.header.frame_id == source_frame and transform.child_frame_id == target_frame:
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
                    transform_matrix = transformations.concatenate_matrices(
                        transformations.translation_matrix(translation), 
                        transformations.quaternion_matrix(quaternion)
                    )
                    return transform_matrix
    return None

def calculate_combined_transform(bag_file):
    base_to_imu_matrix = get_static_transform(bag_file, 'base_link', 'imu_link')
    imu_to_camera_matrix = get_static_transform(bag_file, 'imu_link', 'camera_gray_left')

    if base_to_imu_matrix is None or imu_to_camera_matrix is None:
        raise ValueError("Required static transforms not found in /tf_static.")

    base_to_camera_matrix = np.dot(base_to_imu_matrix, imu_to_camera_matrix)
    return base_to_camera_matrix

def write_kitti_format_file(output_filename, kitti_data):
    with open(output_filename, 'w') as f:
        for transform_matrix in kitti_data:
            kitti_matrix = transform_matrix[:3, :]
            kitti_format = ' '.join(map('{:.12e}'.format, kitti_matrix.flatten()))
            f.write(kitti_format + '\n')

def main(bag_file, output_filename):
    base_to_camera_matrix = calculate_combined_transform(bag_file)

    bag = rosbag.Bag(bag_file)
    kitti_data = []
    world_in_map_matrix = None

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'world':
                world_in_map_matrix = transformations.concatenate_matrices(
                    transformations.translation_matrix([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]),
                    transformations.quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
                )

            if transform.header.frame_id == 'world' and transform.child_frame_id == 'base_link' and world_in_map_matrix is not None:
                base_in_world_matrix = transformations.concatenate_matrices(
                    transformations.translation_matrix([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]),
                    transformations.quaternion_matrix([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
                )

                base_in_map_matrix = np.dot(world_in_map_matrix, base_in_world_matrix)
                base_in_camera_matrix = np.dot(base_in_map_matrix, base_to_camera_matrix)
                kitti_data.append(base_in_camera_matrix)

    write_kitti_format_file(output_filename, kitti_data)
    bag.close()

if __name__ == '__main__':
    main('path_to_your_bag_file.bag', 'output_kitti_file.txt')
