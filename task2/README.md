# README
## Rosbag topics specification
- carto_3dlidar.bag: /tf /tf_statics | aim: /tf_static(gray_camera_left -> base_link) mul /tf(base_link -> map)
- vins_color_fusion.bag: /tf /tf_statics /vins_estimator/odometry | aim: /tf_static(gray_camera_left -> base_link) mul /tf(body -> map)
- vins_color_fusion.bag: /tf /tf_statics /vins_estimator/odometry | aim: /tf_static(gray_camera_left -> base_link) mul /tf(body -> map)
