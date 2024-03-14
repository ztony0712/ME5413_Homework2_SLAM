# ME5413 Homework 2: SLAM

We try four kinds of SLAM algorithms in this task, and record the rosbags. The pcd files of aloam and lego-loam is saved, but the map of cartograher is pgm file. There is no map for the vins-fusion cause its developer didn't provide the corresponding rostopic for saving.

Finally, we write the scripts to extract SLAM results into txt file and evalate them using evo tools. The following is list of all things we provide for each algorithm.

- [x] Extracted txt file
- [x] Map file in the format of pcd/pgm/none
- [x] Evaluation result plot

Furthermore, we provide the scripts to extract the txt file by refering odom and tf topic respectively. 

## Run odom_bag2tum.py
The odom topic is the topic of the odometry message. Different SLAM algorithm may use different odom topic name. Only aloam, lego-loam, and vins-fusion have odom topic.

```python3 odom_bag2tum.py <path/to/yourbag.bag> <path/to/your_tum.txt> --topic </your_odom_topic>```


## Run tf_bag2tum.py
The map frame is the the orinial frame of the map. Different SLAM algorithm may use different map frame name. The cartographer doesn't have odom topic, so we use tf to extract the pose of the map frame.

```python3 tf_bag2tum.py <path/to/yourbag.bag> <path/to/your_tum.txt> --map <your_map_frame>```
