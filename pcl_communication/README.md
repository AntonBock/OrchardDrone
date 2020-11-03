## pcl_communication

Subscribed topic for .pcd files: /cloud_pcd 
(only uses one or the other)
Subscribed topic for live data: /camera/depth/color/points
Published topic: /volumetric_data

# Prerequisites (click for installation guide)
 - 

    
### Executing the code
            
1. Run roscore
```c
roscore
``` 

2.  Run the pcl_communication program 
```c
rosrun pcl_communication pcl_communication
``` 

3. Send .pcd file to the /cloud_pcd topic
```c
rosrun pcl_ros pcd_to_pointcloud dataTree.pcd
```

                
----

### Further improvements

1. Update this READme
2. Clean and comment code
3. Create launch file for running the program
