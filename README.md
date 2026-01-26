# Description
Real-time simulation of a Virtuoso CTR interacting with deformable tissue, with ROS integration. 

# Authors
**Maintainer**: Sam Tobin (stobin2@vols.utk.edu)

## Table of Contents
* [API](#api)
  * [Nodes](#nodes)
    * [Any Simulation](#any-simulation)
    * [`VirtuosoSimulation`](#virtuososimulation)
    * [`VirtuosoCTAnatomySimulation`](#virtuosoctanatomysimulation)
    * [`FixedObjectSimulation`](#fixedobjectsimulation)
  * [API Changes](#api-changes)
* [Installation](#installation)
  * [Hardware Dependencies](#hardware-dependencies)
  * [OS Requirements](#os-requirements)
  * [ALISS Dependencies](#aliss-dependencies)
  * [External Dependencies](#external-dependencies)
  * [Compilation](#compilation)
* [Use](#use)
  * [ROS Launch](#ros-launch)
    * [Launch Files](#launch-files)
    * [Example Usage](#example-usage)
  * [Demos](#demos)
    * [Virtuoso Trachea Demo](#virtuoso-trachea-demo)
    * [Simple grasping demo](#simple-grasping-demo)
    * [Fixed object demo](#fixed-object-demo)
  * [Changing simulation parameters](#changing-simulation-parameters)
    * [Config files](#config-files)
    * [Changing the mesh](#changing-the-mesh)
* [Code structure](#code-structure)
  * [Simulation](#simulation)
  * [Simobject](#simobject)
  * [Solver](#solver)
  * [Config](#config)

# API
## Nodes
`sim_bridge` `sim_bridge`: The ROS interface to the sim. This node, upon startup, will launch a simulation and communicate with it via ROS. The topics subscribed/published to will vary depending on the **type** of simulation that is launched. The types of simulations that can be launched (with examples) can be found here (TODO: link). `sim_bridge` has a general ROS interface that exists regardless of the specific type of simulation.

### Any Simulation
**Publishers:**
In general, the `sim_bridge` node publishes the vertices for any deformable mesh in the simulation. The list of topics can be found below:
| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/sim/output/mesh_vertices_pc_<i>` | N/A | Vertices of the ith deformable mesh in the simulation, as a point cloud. This is primarily for visualization purposes in RViz or Foxglove. | `/sim/world` | `sensor_msgs/PointCloud2` | Purely for visualization purposes. |
| `/sim/output/stiffness_mat_<i>` | N/A | Stiffness matrix for the ith deformable mesh in the simulation. | `/sim/world` | `std_msgs/Float64MultiArray` | Optional, enabled by setting the `publish_matrices` parameter to `true`. Computing the stiffness matrix (right now) is very slow. Should only be enabled for small meshes and when integrating with factor graph. |
| `/sim/output/vertices_mat_<i>` | N/A | Nx3 matrix of current vertex positions for the ith deformable mesh in the simulation. | `/sim/world` | `std_msgs/Float64MultiArray` | Optional, enabled by setting the `publish_matrices` parameter to `true`. |
| `/sim/output/faces_mat_<i>` | N/A | Nx3 matrix of the current **surface** faces for the ith deformable mesh in the simulation. The faces are specified as 3-vectors of vertex indices, 0-indexed. | N/A | `std_msgs/Int32MultiArray` | Optional, enabled by setting the `publish_matrices` parameter to `true`. |
|`/sim/output/elements_mat_<i>` | N/A | Nx4 matrix of current elements for the ith deformable mesh in the simulation. The elements are specified as 4-vectors of vertex indices, 0-indexed. | N/A | `std_msgs/Int32MultiArray` | Optional, enabled by setting the `publish_matrices` parameter to `true. |

**Node parameters:**
Every simulation has the following parameters
| Parameter    | Type | Default | Description |
|--------------|------|---------|-------------|
| `publish_rate_hz` | `double` | 30 | The publishing rate (in Hz) of every topic in `sim_bridge` |
| `publish_matrices` | `bool` | `false` | Whether or not to publish the stiffness matrix, vertices matrix, faces matrix, and elements matrix for each deformable mesh in the simulation. |
| `use_wall_time_for_publishing` | `bool` | `false` | When the simulator ROS node publishes topics, it has two options for the time scale used for publishing: simulated time or wall time. When using simulated time, a 1 Hz publish rate will correspond to publishing once per **simulated** second. This may be faster of slower than 1 Hz wall clock time. Generally, messages should be published using wall clock time for consistent intervals. However, for some messages that are extremely slow to compute (i.e. stiffness matrices), you might prefer that the simulated time is used. |

### `VirtuosoSimulation`
**Publishers:**
Any simulation that involves a Virtuoso robot publishes the following additional topics:
| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/sim/output/arm1_tip_frame` | `/ves/left/joint/measured_cp` | The real (inner tube) tip frame of the simulated Virtuoso left arm. Has both position and orientation. | `ves/left/base` | `geometry_msgs/PoseStamped` | This should align with the definition of `/ves/left/joint/measured_cp`. |
| `/sim/output/arm2_tip_frame` | `/ves/right/joint/measured_cp` | The real (inner tube) tip frame of the simulated Virtuoso right arm. Has both position and orientation. | `ves/left/base` | `geometry_msgs/PoseStamped` | This should align with the definition of `/ves/right/joint/measured_cp`. |
| `/sim/output/arm1_commanded_tip_frame` | `/ves/left/joint/setpoint_cp` | The commanded (inner tube) tip position of the simulated Virtuoso left arm. Contains only position. | `ves/left/base` | `geometry_msgs/PoseStamped` | This should align with the definition of `ves/left/joint/setpoint_cp`. |
| `/sim/output/arm1_commanded_tip_frame` | `/ves/right/joint/setpoint_cp` | The commanded (inner tube) tip position of the simulated Virtuoso right arm. Contains only position. | `ves/left/base` | `geometry_msgs/PoseStamped` | This should align with the definition of `ves/right/joint/setpoint_cp`. |
| `/sim/output/arm1_joint_state` | N/A | The current (simulated) joint state of the Virtuoso left arm. | N/A | `sensor_msgs/JointState` | |
| `/sim/output/arm2_joint_state` | N/A | The current (simulated) joint state of the Virtuoso right arm. | N/A | `sensor_msgs/JointState` | |
| `/sim/output/arm1_frames` | N/A | The current (simulated) coordinate frames along the backbone of the Virtuoso left arm. | `ves/left/base` | `geometry_msgs/PoseArray` | The frames are those at the integration points used in statics model of the Virtuoso arm. |
| `/sim/output/arm2_frames` | N/A | The current (simulated) coordinate frames along the backbone of the Virtuoso right arm. | `ves/left/base` | `geometry_msgs/PoseArray` | |
| `/sim/output/arm1_tool_tip_frame` | N/A | The current (simulated) tip frame of the tool tube of the Virtuoso left arm. Has both position and orientation. | `ves/left/base` | `geometry_msgs/PoseStamped` | This is only publishes when there is actually a tool tube in use (i.e. the palpation tube) that extends past the inner tube. |
| `/sim/output/arm2_tool_tip_frame` | N/A | The current (simulated) tip frame of the tool tube of the Virtuoso right arm. Has both position and orientation. | `ves/left/base` | `geometry_msgs/PoseStamped` | | 
| `/sim/output/arm1_net_force` | N/A | The current (simulated) net force on the Virtuoso left arm. | `ves/left/base` | `geometry_msgs/Vector3Stamped` | The net force is determined by summing all of the forces from individual collisions between the Virtuoso arm and the tissue. |
| `/sim/output/arm2_net_force` | N/A | The current (simulated) net force on the Virtuoso right arm. | `ves/left/base` | `geometry_msgs/Vector3Stamped` |  |
| `/sim/output/trachea_partial_view_pc` | N/A | A partial view point cloud from the endoscope camera view of just the trachea. | `ves/left/base` | `sensor_msgs/PointCloud2` | Optional. Enabled by setting the `partial_view_pc` parameter to `true`. Requires some computation time, since the Embree ray collision scene must be updated. Also requires that a tissue mesh be in the scene that has labels for trachea and tumor parts. |
| `/sim/output/tumor_partial_view_pc` | N/A | A partial view point cloud from the endoscope camera view of just the tumor. | `/ves/left/base` | `sensor_msgs/PointCloud2` | Optional. Same as above. |

**Subscriptions:**
Any simulation that involves a Virtuoso robot subscribes to the following topics for inputting commandes to the robot. The interface was designed to try to be identical to that of `ves_ros_interface`. (i.e. controlling the simulated Virtuoso arm over ROS should be the same as controlling the real Virtuoso arm over ROS).
| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/sim/input/arm1_joint_state` | `/ves/left/joint/servo_jp` | Input joint state for the simulated Virtuoso left arm. Expects the same format as in `ves_ros_interface`. | N/A | `sensor_msgs/JointState` | |
| `/sim/input/arm2_joint_state` | `/ves/right/joint/servo_jp` | Input joint state for the simualted Virtuoso right arm. Expects the same format as in `ves_ros_interface`. | N/A | `sensor_msgs/JointState` | |
| `/sim/input/arm1_tip_pos` | `/ves/left/joint/servo_cp` | Input commanded tip position for the simulated Virtuoso left arm. Only position will be used. | `ves/left/base` | `geometry_msgs/PoseStamped` | |
| `/sim/input/arm2_tip_pos` | `/ves/right/joint/servo_cp` | Input commanded tip position for the simulated Virtuoso right arm. Only position will be used. | `ves/left/base` | `geometry_msgs/PoseStamped` | |
| `/sim/input/arm1_tool_state` | `/ves/left/set_tool` | Input tool state for the simulated Virtuoso left arm. Same as `ves_ros_interface`. | N/A | `std_msgs/Int8` | 0 = off, 1 = on |
| `/sim/input/arm2_tool_state` | `/ves/right/set_tool` | Input tool state for the simualte Virtuoso right arm. Same as `ves_ros_interface`. | N/A | `std_msgs/Int8` | 0 = off, 1 = on |

### `VirtuosoCTAnatomySimulation`
**Publishers:**
Any simulation that involves a Virtuoso robot interacting with anatomical meshes has the following additional publishers:
| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/sim/output/arm2_net_force` | N/A | The current (simulated) net force on the Virtuoso right arm. | `ves/left/base` | `geometry_msgs/Vector3Stamped` |  |
| `/sim/output/trachea_partial_view_pc` | N/A | A partial view point cloud from the endoscope camera view of just the trachea. | `ves/left/base` | `sensor_msgs/PointCloud2` | Optional. Enabled by setting the `partial_view_pc` parameter to `true`. Requires some computation time, since the Embree ray collision scene must be updated. Also requires that a tissue mesh be in the scene that has labels for trachea and tumor parts. |
| `/sim/output/tumor_partial_view_pc` | N/A | A partial view point cloud from the endoscope camera view of just the tumor. | `/ves/left/base` | `sensor_msgs/PointCloud2` | Optional. Same as above. |

**Node parameters:**
The `VirtuosoCTAnatomySimulation` has the following extra parameters, mostly for configuring the partial view point clouds:
| Parameter    | Type | Default | Description |
|--------------|------|---------|-------------|
| `partial_view_pc` | `bool` | `true` | Whether or not to publish the partial view point clouds from the endoscope camera view. |
| `partial_view_pc_hfov` | `double` | 80 | The horizontal field-of-view (in degrees) of the rays cast for computing the point cloud. I.e. for 80 degrees, rays will be cast between [-40, +40] degrees in angle left to right. |
| `partial_view_pc_vfov` | `double` | 30 | The vertical field-of-view (in degrees) of the rays cast for computing the point cloud. |
| `partial_view_pc_sample_density` | `double` | 1 | The density of rays (per degree) cast when computing the point cloud. Higher number = denser point cloud. Note: this doesn't really correspond to anything physical (like sensor parameters), it's just a way to get a denser point cloud if you want it. |
| `CT_frame_name` | `string` | `CT/kuka` | The name of the CT origin frame in the `tf` tree. The simulation listens to the transform between `CT_frame_name` and `ves/left/base` to dictate the position of meshes in the simulation. |


### `FixedObjectSimulation`
**TODO**


## API Changes
None.

# Installation
## Hardware Dependencies
None.

## OS Requirements
As of right now, I have only tested installation, compilation and execution on Ubuntu 24.04. I'm sure other Linux distros would work, it just might take some effort in finding the right packages.

## ALISS Dependencies
None.

## External Dependencies
This repository relies on a few external dependencies, namely:
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) for processing input YAML files
- [Eigen](https://gitlab.com/libeigen/eigen) for matrix/vector operations
- [Easy3D](https://github.com/LiangliangNan/Easy3D) as a visualization option
- [GMSH](https://gitlab.onelab.info/gmsh/gmsh) for creating tetrahedral meshes from .obj or .stl files
- [Mesh2SDF](https://github.com/smtobin/Mesh2SDF) for creating SDFs from arbitrary surface meshes
- [embree](https://github.com/RenderKit/embree) for BVH building and ray/closest point queries
- [VTK](https://gitlab.kitware.com/vtk/vtk) as a visualization option
- OpenHaptics drivers for interfacing with Geomagic Touch haptic devices

The dependency installation and environment variables setup is done with the script located in `scripts/install.sh`. Example usage:
```
bash scripts/install.sh
```
`sudo` may be needed to install the required `apt` packages. A directory `dependencies/` is created at the root level of the repository to store the external dependencies. The script clones the latest version of all dependencies, putting the code in the `dependencies/src` directory. Each library is built (in `Release` mode) and installed **locally** in the `dependencies/install` directory.

## Compilation
**Building the simulator:**

Once the dependencies are installed, building should be as simple as
```
mkdir build/ && cd build
cmake .. -DNO_GPU=1
make -j12
```
Since the GPU implementation is broken currently, use `DNO_GPU=1` to not build the GPU part of the code (this is only necessary if you have CUDA installed on your system).

You can run a first test with
```
./Test ../config/config.yaml
```
See [Demos](#demos) for other demos to test out.


**To initially build the ROS nodes:**

If wanting to use ROS, it is assumed that is already installed on your system. Installation instructions for Ubuntu 24.04 can be found [here](https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html).

Some of the launch files simulatenously launch a Rosbridge server for connecting to Foxglove for visualization. This can be installed with
```
sudo apt-get install ros-jazzy-rosbridge-server -y
```

When you build the simulator from the `build/` directory, CMake config files are automatically generated each time (inside the `build/` directory) that point CMake to the appropriate library files that are also inside the `build/` directory. The CMake file for the ROS node automatically knows where these files are, so all that is needed is to run
```
colcon build
```
from the `ros_workspace/` directory.

**IMPORTANT:** Whenever you rebuild anything in the simulator (i.e. you pulled new changes or made edits yourself and needed to run `make`) and you want the most recent changes to be reflected in the ROS node, **you must**:
- remove the `build/` folder in `ros_workspace`
- run `colcon build`

# Use
## ROS Launch
### Launch Files

Two launch files are provided:
* `ros_workspace/launch/sim_bridge_with_rosbridge_server.launch.py` - generic launch file. Launches the appropriate `SimBridge` ROS node and starts a `rosbridge` WebSocket connection on port 9090 (useful for visualizing with Foxglove).
* `ros_workspace/launch/virtuoso_sim_bridge.launch.py` - launch file specifically for `VirtuosoCTAnatomySimulation`. Includes topic remappings for Virtuoso topics, extra parameters for configuring partial view point clouds, etc.

The launch files provide a few launch arguments:
* Launch argument `config_filename` - the absolute path to the config filename to be used to launch the simulation. Default: `/worksapce/config/demos/virtuoso_trachea/virtuoso_trachea.yaml`.
* Launch argument `simulation_type` - the "type" of simulation to be launched. Corresponds to the camel-case class name of the type of simulation to be launched. Default: `VirtuosoCTAnatomySimulation`. Other options: `GraspingSimulation`, `VirtuosoSimulation`, `Simulation`.

### Example usage

Launching the Virtuoso robot + trachea demo:
```
ros2 launch launch/virtuoso_sim_bridge.launch.py
```
To use a different config file with the trachea demo (e.g. a config file that loads a prostate mesh):
```
ros2 launch launch/virtuoso_sim_bridge.launch.py config_filename:=<path to config .yaml file>
```
Launching the simple grasping demo:
```
ros2 launch launch/sim_bridge_with_rosbridge_server.launch.py config_filename:=../config/demos/simple_grasping/grasping_config.yaml simulation_type:=GraspingSimulation
```

## Demos
Below describes various off-the-shelf demos that demonstrate some of the simulator capabilities.

### Virtuoso trachea demo
This demo consists of a life-size trachea mesh with a tumor and a Virtuoso robot that can interact with the tissue mesh. The yellow sphere at the tip of the robot represents the grasping radius -- when grasping is toggled to be active, all tissue mesh nodes inside the sphere will be "grasped" and move around with the tip of the robot. When grasping is toggled to be inactive, any grasped mesh nodes are released.

**Running the demo:**
```
./VirtuosoTracheaDemo ../config/demos/virtuoso_trachea/only_tumor.yaml
```

**Controls:**
Different input types can be used to control the Virtuoso robot. The options are "Keyboard", "Mouse", and "Haptic" (Geomagic Touch). The input type can be changed by changing the `input-device` field in the config file.

For all simulations: press `Alt` to toggle which Virtuoso arm is actively being controlled (only applicable when there is more than one arm). Press `Tab` to switch to the endoscope view.

_Keyboard_: Controls the joint variables directly. `Q/A` = inner tube rotation; `W/S` = outer tube rotation; `E/D` = inner tube translation; `R/F` = outer tube translation. `Space` = toggle grasping.

_Mouse_: Controls the tip of the robot (joint variables computed through inverse kinematics). Hold `Space` and move the mouse to move the active arm tip position parallel to the camera plane. Hold `Space` and scroll the mouse wheel to move the active arm tip position into/out of the camera plane. `Left click` toggles grasping (note: you do not need to hold `Left click` to continually grasp -- one mouse click to toggle on, and another mouse click to toggle off).

_Haptic_: Controls the tip of the robot. Hold the first button and move the input device to move the active arm tip position. Press and hold the second button to grasp (note: you need to continually hold the second button to keep grasping -- hold both buttons to grasp and move at the same time).

**Screenshots:**

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/2a1d9b68-c79e-40c7-8379-1f263de73069"/>

**Config file parameters:**

- _CT-to-VB-transform-translation_: the translation part of the transfrom from `CT -> VB` (i.e. from the CT origin frame to the base of the left Virtuoso arm). Note that the **"robotics"** convention is used here, meaning that the `CT -> VB` transform will transform a point expressed in the CT frame into a point expressed in the VB frame.
- _CT-to-VB-transform-rotation_: the rotational part of the transform from `CT -> VB`, expressed as XYZ Euler angles.

In the config file, just the `CT -> VB` transform should dictate the positions of the meshes (if the meshes were created from CT scans). No positional/rotational offsets for individual objects should be necessary.

### Simple grasping demo
This demo is basically a simpler version of the above demo, without the Virtuoso arm. The yellow sphere at the tip of the robot represents the grasping radius -- when grasping is toggled to be active, all tissue mesh nodes inside the sphere will be "grasped" and move around with the tip of the robot. When grasping is toggled to be inactive, any grasped mesh nodes are released. The bottom face of the mesh is optionally fixed to better see the effects of deformation.

**Running the demo:**
```
./GraspingTest ../config/demos/simple_grasping/grasping_config.yaml
```

**Controls:**
Hold `Space` and move the mouse to move the active arm tip position parallel to the camera plane. Hold `Space` and scroll the mouse wheel to move the active arm tip position into/out of the camera plane. `Left click` toggles grasping (note: you do not need to hold `Left click` to continually grasp -- one mouse click to toggle on, and another mouse click to toggle off). Press `W` to increase the grasp radius, and press `S` to decrease the grasp radius. Note that the amount the grasping sphere moves per frame depends scales with the size of it.


### Fixed object demo
This is a very simple demo that fixes part of an object's surface and lets the rest deform under gravity. It also gives the user the ability to periodically save the vertices and stiffness matrix at a fixed interval of time steps, and the demo also has a ROS interface. Over this ROS interface, a partial-view point cloud (generated from a user-specified vantage point) can be sent.

**Running the demo:**
```
./FixedObjectSim ../config/demos/fixed_object/banana.yaml
```
or
```
./FixedObjectSim ../config/demos/fixed_object/fixed_cube.yaml
```

**Config file parameters:**

- _cube-fixed-face_: the "face" (of the object) to fix. The options are "none", "left", "right", "top", "bottom". For the "left" option, this finds all vertices on the object with the minimum Y-coordinate and sets those to be fixed. Similar analogous operations are performed for the "right", "top" and "bottom" options. Note that this is really only well-defined for simple polyhedra like a cube, and for arbitrary objects you will need to explicitly specify which vertices/faces to fix.
- _text-file-save-interval_: the (integer) number of time steps between the generation of text files for the vertices and the stiffness matrix. When set to an integer < 0, no text files will be saved.
- _text-file-save-folder_: the string path for where to save the output text files. This folder will be created if it does not already exist.
- _point-cloud-sample-position_: the position [X,Y,Z] of where the partial-view point cloud should be generated from.
- _point-cloud-sample-orientation_: the orientation (specified using the XYZ Euler angle convention) of the partial-view point cloud "view" frame. The Z-axis of the view frame is the center of the partial view point cloud.
- _fixed-faces-filename_: (optional) a path to a specially formatted .txt file that specifies the vertices/faces to fix in the object. This gives the user the ability to explicity specify which vertices/faces are fixed for arbitrary objects. More info below.

**Generating the fixed faces file:**
- **Step 0**: _Generating the .msh file._ Take your initial surface mesh (.stl or .obj) file and substitute it into the config file (i.e. edit the `filename` parameter for an object). When you run the demo, the simulator will automatically generate a volumetric .msh file (with the same name) using GMSH. It will also generate a `<filename>_surface_mesh.obj` file that is just the surface of the volumetric .msh file.
- **Step 1**: _Selecting fixed faces with MeshLab._ Open the `<filename>_surface_mesh.obj` file in MeshLab (note: it is important to use the generated .obj file because GMSH will sometimes change the ordering of the surface faces when it does volumetric mesh generation). Using the "Select faces in rectangular region" tool, select the faces that you want to fix.
- **Step 2**: _Generating .ply file with fixed face information._ With the faces you want fixed selected, run Filters --> Quality Measure and Computations --> Per Face Quality Function, and type the expression `fsel*fi` in the user-defined function box. This assigns to each face a quality value of the face index if it is selected, and 0 if not. Then, export the mesh as an ASCII .ply file. Check the box for "Quality", and uncheck the box for "Binary Encoding".
- **Step 3**: _Generating the fixed faces .txt file._ Run the following command to extract only the lines from the .ply file that correspond to the fixed faces:
```
cat <filename>.ply | grep -v " 0 $" | grep "^3 " > fixed_faces.txt
```
Now `fixed_faces.txt` should only have faces that are fixed with the following format:
```
3 <v1> <v2> <v3> <f>
```
This is precisely the format that the simulator is expecting the text file under `fixed-faces-filename` to have.

## Changing simulation parameters
This section described how to change simulation parameters to suit your needs.

### Config files
This is the first place you should go to play with simulation parameters. Config files are `.yaml` files that are parsed on simulation launch and used to set parameters of the simulation and all the objects in the simulation. This means that we can change parameters in a config file (e.g. the time step or the size of an object) **without having to rebuild**.

Provided config files are found in the `config/` directory. A commented example config file that should cover most of the options available in config files can be found [here](https://github.com/smtobin/XPBD_Sandbox/blob/1952ffcee7a9623914602a4baf0c7eccfefcebf0/config/example_config.yaml).

### Changing the mesh
The mesh of a simulated deformable object can be changed by simply changing the `filename` parameter. For deformable objects, a volumetric mesh is required (common mesh formats like `.stl` and `.obj` are for surface meshes). Internally, the simulation uses GMSH's mesh format (`.msh`). The simulation is able to take input `.stl` and `.obj` surface mesh files and use GMSH to generate a volumetric `.msh` mesh file from that. A `.msh` file with the same filepath as the input `.stl` or `.obj` file will be generated.

# Code Structure
This section will briefly go over how the repository is organized, and generally how the code is structured.

### Simulation
The `Simulation` class (found in the `simulation/` folder) is the owner/manager of everything in a simulation. It is responsible for creating and updating the objects in the sim and the visualization (if enabled). Executable files (e.g., `exec/main.cpp`) use the `run()` method to start the simulation, which will first call the `setup()` method, then spawn a thread that will perform the updates (i.e. it repeatedly calls the `update()` method which steps forward in time), and lastly launch the visualization.

Various simulation functionalities are delegated to specific "scenes" to abstract their functionality. For example, the child `GraphicsScene` object handles the visualization aspects of the simulation, and the child `CollisionScene` object handles the collisions between objects. Note that these do not have to be used in every simulation (i.e. graphics/collisions can be turned off), and not all objects in the simulation have to be added to each scene (i.e. certain things may not be used in collision or visualization).

There are multiple derived `Simulation` classes that augment the functionality to aid in simulating specific scenarios. For example, `BeamSimulation` cantilevers an object and measures its deflection.

### SimObject
The `simobject/` folder contains all the simulation objects which move around and interact with one another, all inheriting from the abstract base `Object` class.

### Solver
The `solver/` folder contains everything to do with projecting constraints using the XPBD algorithm.

The `Constraint` abstract base class implements a generic interface for a constraint (which does not have to necessarily be used with the XPBD algorithm). This interface basically revolves around two functions: `evaluate()` which evaluates the constraint function `C(x)`, and `gradient()` which evaluates the constraint gradient with respect to the positions that make up that constraint. All classes derived from `Constraint` (e.g. `HydrostaticConstraint`, `DeviatoricConstraint`) implement these functions enabling the generic evaluation of a constraint and its gradient.

The `ConstraintProjector` class implements the XPBD projection algorithm (i.e. solving Equation (16) in the XPBD paper for $\Delta\lambda$), and outputs the resulting position updates ($\Delta \mathbf{x}$). This is implemented in the `project()` class method. The `CombinedConstraintProjector` class projects two constraints simultaneously.

The `XPBDSolver` class solves the constraints with its `solve()` class method. It owns a list of `ConstraintProjector`s that it will use to iteratively solve the constraints - i.e. by calling the `project()` method for each `ConstraintProjector` and applying the position updates. How the `XPBDSolver` applies the position updates is up to the derived classes, one solver iteration is contained in the protected method `_solveConstraints()`. For example, `XPBDGaussSeidelSolver` uses a Gauss-Seidel update strategy (as described in the XPBD paper) which updates the positions immediately after projecting each constraint.

### Config
The `include/config/` folder contains the `Config` class and its derived classes that parse YAML config files and make the parameters available for object instantiation. The base `Config` class provides the machinery needed (with the help of the [yaml-cpp](https://github.com/jbeder/yaml-cpp) library) to parse a YAML node/subnode and extract expected parameters from it using the `_extractParameter()` templated function. Static default values can be set and used if the YAML parameter is not found. If the extracted parameter is limited to a set of options (i.e. it should be a choice from an enum), `_extractParameterWithOptions()` can be used instead, and we pass in a static map that maps user-specified text to the appropriate value (e.g. from "Gauss-Seidel" to `XPBDObjectSolverType::GAUSS_SEIDEL`).

Classes derived from `Config` correspond to specific objects and have extract additional parameters that correspond specifically to the options of that type of object. For example, `MeshObjectConfig` extract information from the YAML file that is used to set up a `MeshObject`, such as the size, initial position, initial velocity, color, etc.
