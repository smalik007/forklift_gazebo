1. clone the project to a new folder_name
2. change the "CATKIN_WORKSPACE_ROOT_PATH" in source file to your specified folder_name path
3. open model.sdf file at src/forklift_gazebo/models/fork_lift_shift/ and change all the path of stl files as per your folder_name path
4. from your folder_name root path run $catkin_make
5. run $source source
6. run $roslaunch forklift_gazebo forklift.launch
7. In new tab goto src/forklift_gazebo/src/
8. run $python joint_state_publisher.py
