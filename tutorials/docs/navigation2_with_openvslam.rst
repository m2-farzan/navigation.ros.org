.. _navigation2-with-openvslam:

Using an External SLAM Engine (Open-VSLAM)
******************************************

- `Overview`_
- `Requirements`_
- `Tutorial Steps`_

Overview
========

Navigation2 can be used with any SLAM engine.
The only requirement is that the engine must provide the ``map->odom`` transformation.
So in this tutorial we're going to use OpenVSLAM (community version) instead of SLAM Toolbox.
OpenVSLAM performs SLAM using one or more cameras instead of a LIDAR.

Before starting this tutorial, completing the :ref:`getting_started` is recommended.

Requirements
============

1- Navigation2
--------------

If you don't have Navigation2 installed, please follow :ref:`getting_started`.

2- A Robot with Camera
----------------------

For OpenVSLAM to work, we need to have cameras on the robot.
In this tutorial we'll assume a stereo setup (i.e. two cameras), but the choice is not conceptual.
To learn about other configurations, see `OpenVSLAM-ROS docs <https://github.com/OpenVSLAM-Community/openvslam_ros>`_.

In a stereo setup, the cameras should publish to ``/camera/left/image_raw`` and ``/camera/right/image_raw`` topics.
If you're using an actual robot with actual USB cameras,
you can achieve this using ``cam2image`` node from ``ros-<distro>-image-tools`` package in conjuction with ``republish`` node from ``ros-<distro>-image-transport`` package.
In a simulation, you just need to define the cameras in the URDF using the following lines:

.. code-block:: XML

  <xacro:property name="camera_xoff" value="0.12" />
  <xacro:property name="camera_yoff" value="0.2" />
  <xacro:property name="camera_zoff" value="0.2" />

  <xacro:macro name="camera_unit" params="side y_reflect">
      <link name="camera_${side}_link">
          <visual>
              <origin xyz="0 0 0" rpy="0 ${pi/2} 0" />
              <geometry>
                  <cylinder radius="0.02" length="0.08" />
              </geometry>
              <material name="Gray">
                  <color rgba="0.5 0.5 0.5 1.0" />
              </material>
          </visual>
          <xacro:cylinder_inertia m="0.5" r="0.10" h="0.04" />
      </link>

      <joint name="camera_${side}_joint" type="fixed">
          <parent link="base_link" />
          <child link="camera_${side}_link" />
          <origin xyz="${camera_xoff} ${y_reflect * camera_yoff} ${camera_zoff}" rpy="0 0 0" />
      </joint>

      <gazebo reference="camera_${side}_link">
          <sensor type="camera" name="camera_${side}">
              <always_on>true</always_on>
              <update_rate>15.0</update_rate>
              <camera name="camera_${side}">
                  <horizontal_fov>1.46608</horizontal_fov>
                  <image>
                      <width>320</width>
                      <height>180</height>
                      <format>R8G8B8</format>
                  </image>
              </camera>
              <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
                  <ros>
                      <namespace>camera</namespace>
                      <qos>
                          <topic name="/camera/${side}/image_raw">
                              <publisher>
                                  <reliability>reliable</reliability>
                              </publisher>
                          </topic>
                      </qos>
                  </ros>
                  <camera_name>${side}</camera_name>
                  <hack_baseline>0.2</hack_baseline>
              </plugin>
          </sensor>
      </gazebo>
  </xacro:macro>

  <xacro:camera_unit side="left" y_reflect="1" />
  <xacro:camera_unit side="right" y_reflect="-1" />


Tip: You can verify the images are being published by launching `rqt` and navigating to `Plugins -> Visualization -> Image View`.

OpenVSLAM
---------

First, note that OpenVSLAM is a standalone C++ library while OpenVSLAM-ROS is a ROS package built on top of OpenVSLAM.

OpenVSLAM doesn't come with official binary releases.
If you happen to be using Ubuntu 20.04 64-bit, you can use `this unofficial OpenVSLAM package <https://github.com/m2-farzan/openvslam-ubuntu-package>`_.
If you're on some other system or you just prefer to build OpenVSLAM yourself, follow the instructions in their `documentation <https://openvslam-community.readthedocs.io/en/latest/installation.html>`_.

After installing OpenVSLAM, it's time to build OpenVSLAM-ROS. The following commands should do it:

.. code-block:: Bash

  $ git clone --branch ros2 --depth 1 https://github.com/OpenVSLAM-Community/openvslam_ros.git
  $ colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/opt/openvslam-community -DUSE_PANGOLIN_VIEWER=ON

OpenVSLAM also provides a Docker-based installation option documented `here <https://github.com/OpenVSLAM-Community/openvslam_ros/blob/ros2/doc/tutorial.md>`_.


Tutorial Steps
==============

1- Obtain a Bag of Words
------------------------

OpenVSLAM requires a `Bag of Words` file to work.
It is a pre-trained set of `words` used for storing the visual elements of the places visited by the robot.
This way VSLAM knows when it's revisiting a previously mapped location, achieving `loop closure`.

For this tutorial we can just think of Bag of Words as a file we need to download.
OpenVSLAM provides it in `this link <https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`_.
Save the file as ``orb_vocab.fbow``.

Note: If you've built OpenVSLAM yourself with DBoW2 option (instead of FBoW), you need to download `this file <https://github.com/OpenVSLAM-Community/DBoW2_orb_vocab/raw/main/orb_vocab.dbow2>`_ instead.

2- Create OpenVSLAM Config File
-------------------------------

OpenVSLAM needs a config file too.
It's a YAML file that defines the camera setup and VSLAM tuning parameters.

Start with the following minimal config file.
If you're using a real robot, replace the camera specs with that of your camera.
Refer to this `page <https://openvslam-community.readthedocs.io/en/latest/parameters.html>`_ to learn more about these parameters.

.. code-block:: YAML

  Camera:
    name: "My Virtual Camera"
    setup: "stereo"
    model: "perspective"
    fps: 15.0
    cols: 320
    rows: 180
    color_order: "RGB"
    fx: 177.70
    fy: 177.70
    cx: 160
    cy: 90
    k1: 0
    k2: 0
    p1: 0
    p2: 0
    k3: 0
    focal_x_baseline: 71.08

  Preprocessing:
    max_num_keypoints: 1000

  Feature:
    name: "feature extraction config"
    scale_factor: 1.2
    num_levels: 8
    ini_fast_threshold: 20
    min_fast_threshold: 7

  Mapping:
    baseline_dist_thr_ratio: 0.02
    redundant_obs_ratio_thr: 0.95

Save the file as ``config.yaml``.

Tip:

.. math::

  f = f_x = f_y = \frac{\text{image_width}}{2 \tan(\text{FOV}/2)}

.. math::

  \text{focal_x_baseline} = f * \text{Cameras Distance} \qquad \text{(Only applicable to stereo)}

.. math::

  c_x = \frac{\text{image_width}}{2} \qquad c_y = \frac{\text{image_height}}{2}

Tip: You can find more config file examples `here <https://github.com/OpenVSLAM-Community/openvslam/tree/main/example>`_.


3- Launch Robot
---------------

Start the real or simulated robot.
Make sure the camera images are being published to the correct topics.

Tip:
Avoid plain/empty environments as they lack the visual elements required for VSLAM to work well.
In gazebo, use ``/usr/share/gazebo-11/worlds/willowgarage.world`` for this tutorial.

4- Launch Navigation2
---------------------

Launch Navigation without nav2_amcl and nav2_map_server.

  ``ros2 launch nav2_bringup navigation_launch.py``

If the robot is running in a simulation:

  ``ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True``

Since VSLAM is not up yet, you may see some warnings indicating `map` frame does not exist.

5- Launch OpenVSLAM
-------------------

Launch OpenVSLAM in SLAM Mode:

  ``ros2 run openvslam_ros run_slam -v orb_vocab.fbow -c config.yaml --map-db map.msg``

If the robot is in a simulation:

  ``ros2 run openvslam_ros run_slam -v orb_vocab.fbow -c config.yaml --map-db map.msg --ros-args --param use_sim_time:=True``

Alternatively, if you have already generated a map, you can run OpenVSLAM in localization mode:

  ``ros2 run openvslam_ros run_localization -v orb_vocab.fbow -c config.yaml --map-db map.msg``

6- Working with SLAM
--------------------

Open Rviz2 to see robot in the map and navigate to different places.

Move your robot by requesting a goal through RViz or the ROS 2 CLI, ie:

.. code-block:: bash

  ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

You should see the map update live! To save this map to file:

  ``ros2 run nav2_map_server map_saver_cli -f ~/map``

.. image:: images/Navigation2_with_SLAM/navigation2_with_slam.gif
    :width: 700px
    :alt: Navigation2 with SLAM
    :align: center
