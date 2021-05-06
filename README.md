# tams_pick_and_place_pointnetgpd

This package is modified from [tams_ur5_pick_place](https://github.com/TAMS-Group/tams_ur5_pick_place)
and [moveit_gpd_pick_object](https://github.com/TAMS-Group/moveit_gpd_pick_object) to adjust for PointNetGPD grasps.

## Package needed:
- [PointNetGPD](https://github.com/lianghongzhuo/PointNetGPD)
- [camera_positioner](https://github.com/TAMS-Group/camera_positioner)
- [gpd_grasp_msgs](https://github.com/TAMS-Group/gpd_grasp_msgs)

## Usage:
- Prepare a robot with `MoveIt!` configuration, it should contain two move groups, one for the arm, one for the gripper.
- `PointNetGPD` will publish grasps candidates using grasp messages defined at `gpd_grasp_msgs`.
- `tams_pick_and_place_pointnetgpd` will subscribe this message and plan a collision free path to the grasp using `MoveIt!`.
