# Scene segmentation

### Usage

Find plane and segment objects
```
rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_find_plane
```


Find plane and segment objects
```
rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_segment

rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String e_stop
```

Subscribe to the following topics:
Object list:
```
/mir_perception/scene_segmentation/output/object_list
```

Bounding Boxes (for visualization in Rviz)
```
/mcr_perception/scene_segmentation/output/bounding_boxes
```

Object labels (for visualization in Rviz)
```
/mcr_perception/scene_segmentation/output/labels
```

Debug pointcloud (shows filtered input to plane segmentation)
```
/mir_perception/scene_segmentation/output/debug_cloud
```

Object clusters
```
/mcr_perception/scene_segmentation/output/tabletop_clusters
```

Workspace height:
```
/mcr_perception/scene_segmentation/output/workspace_height
```

# Crate segmentation

Publishes bounding boxes and a pose array of objects inside a crate.
The poses are on **top** of the object, not the center, so for grasping you may want
to go down a bit.
An outer bounding box of the crate is also available, but not published.

Launch:

```
roslaunch mir_object_segmentaiton crate_segmentation.launch
```

Start:
```
rostopic pub /mir_perception/crate_segmentation/event_in std_msgs/String e_start
```

Stop:
```
rostopic pub /mir_perception/crate_segmentation/event_in std_msgs/String e_stop
```

Bounding boxes:
```
/mir_perception/crate_segmentation/output/bounding_boxes
```

Poses:
```
/mir_perception/crate_segmentation/output/object_poses
```

