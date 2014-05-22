cmake_minimum_required(VERSION 2.8.3)
project(ptam_com)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

#Declare build dependencies
find_package(catkin REQUIRED 
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

add_message_files(DIRECTORY msg
	FILES
	KeyFrame_msg.msg
	OctoMapPointArray.msg
	OctoMapPointStamped.msg
	OctoMapScan.msg
	Vector3Array.msg
	ptam_info.msg
)

add_service_files(DIRECTORY srv
	FILES
	KeyFrame_srv.srv
	PointCloud.srv
)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#Declare package run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})
