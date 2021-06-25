# pcl_clustering

## Frameworks used
* ROS Melodic
* PCL 1.8

## Methods used
* Voxel Grid Filtering
* RANSAC Planar Segmentation
* Extracting Indices
* Creating KD Tree object for search method
* Euclidean Cluster Extraction



###  Clone Repository
``` 
git clone https://github.com/dishitamidha/pcl_clustering.git
```

###  Build Workspace
``` 
catkin_make 
```

###  Launch Simulation
``` 
roslaunch pcl_task pcl.launch 
```

###  Run PCL clustering node
```
rosrun pcl_task pcl_task_main 
```



![Screenshot from 2021-06-25 20-30-20](https://user-images.githubusercontent.com/75261680/123444261-55041700-d5f4-11eb-9486-e4fe5882ed46.png)
