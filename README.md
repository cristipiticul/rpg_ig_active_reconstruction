Object Classifier using 3D Probabilistic Maps and Multiple Views
================================================================

This repository is an extension of the [Information Gain Based Active Reconstruction](https://github.com/uzh-rpg/rpg_ig_active_reconstruction) project, which is used to create a 3D map an environment (a room, for example), using a mobile robot having a 3D camera.

In this implementation, we create a 3D map of a small object, using a [2-handed robot](http://www.rethinkrobotics.com/baxter/). One  hand moves the camera, while the other hand moves the object.

The main additions to the initial repository are:
- The baxter_ig_interface package, for robot control and saving the viewing poses
- The baxter_pointcloud_correction package, for reducing small displacements in point cloud data
