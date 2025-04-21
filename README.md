# Agro 5G Vineyard 3D reconstruction and multiespectral imagery acquisition

Packages for **a5g_wine_rec** repository! This project focuses on developing cutting-edge algorithms and software tools for precision agriculture using ROS.


## agro5g_arm
Main launch for Z1 arm, Micasense multispectral camera, LiDAR, reconstruction and front camera.  
To launch everything:  
`roslaunch agro5g_arm arm_complete.launch`

## gps_nav
Package for save Georeferenced coordinates for use in navigation tasks  
`roslaunch gps_nav gps_save.launch`

## laser3d_scan
Package for moving arm, lidar acquisition, 3D PointCloud reconstruction and PCD save  
`rosrun laser3d_scan localpc_generator.py `  
`rosrun laser3d_scan data_acquisition_light.py `

Node to FTP upload images  
`rosrun laser3d_scan ftp_upload.py `

## micasense_ros
Package for Multiespectral camera image acquisition and save in JPEG format  
`rosrun micasense micasense_stream_light.py`
`rosrun micasense spectral_save.py`

## ntrip_client
Forked package for ntrip communication  
`roslaunch ntrip_client ntrip_client.launch `

## optrx_sensor
Package for serial acquisition of data for OptRX sensor  
`rosrun optrx_sensor optrx.py`

For the front camera  
`rosrun optrx_sensor front_camera.py`

## urg_node
Working package for the URG-04LX liDAR  
`roslaunch urg_node urg_lidar.launch`

