import rospy
import rosbag
from tf2_msgs.msg import TFMessage
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
                # 转换四元数到矩阵
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
                
                # 提取平移和四元数
                trans = transformations.translation_from_matrix(mat_world_to_base_link)
                rot = transformations.quaternion_from_matrix(mat_world_to_base_link)

                # 转换时间戳格式
                time = transform.header.stamp.to_sec()
                if start_time is None:
                    start_time = time
                # 重置时间戳使得它从0开始
                time -= start_time

                # 添加到数据中
                tum_data.append([time] + list(trans) + list(rot))

    # 将TUM格式数据写入文件
    write_tum_format_file(output_filename, tum_data)

    bag.close()

if __name__ == '__main__':
    main('vins_gray_fusion.bag', 'vins_gray_fusion_tum.txt')
