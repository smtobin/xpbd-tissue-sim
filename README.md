# XPBD_Sandbox
A place to prototype and test algorithms and approaches for simulation of highly deformable elastic materials.

## Table of Contents
* [Building and running a first simulation](#building-and-running-a-first-simulation)
  * [Linux](#linux)
  * [Linux (without Docker)](#linux-without-docker)
  * [Windows](#windows)
* [Changing simulation parameters](#changing-simulation-parameters)
  * [Config files](#config-files)
  * [Changing the mesh](#changing-the-mesh)
  * [Creating a new derived `Simulation` class (advanced)](#creating-a-new-derived-simulation-class-advanced)
* [Demos](#demos)
  * [Virtuoso trachea demo](#virtuoso-trachea-demo)
  * [Simple grasping demo](#simple-grasping-demo)
  * [Fixed object demo](#fixed-object-demo)
* [ROS interface](#ros-interface)
  * [Example usage](#example-usage)
  * [`sim_bridge` with VirtuosoSimulation](#sim_bridge-with-virtuososimulation)
  * [`sim_bridge` in general](#sim_bridge-in-general)
* [Modifying the code](#modifying-the-code)
* [Code structure](#code-structure)
  * [Simulation](#simulation)
  * [Simobject](#simobject)
  * [Solver](#solver)
  * [Config](#config)

## Building and running a first simulation
Different `Dockerfile`s and Docker compose files have been provided that will set up an interactive Docker container in which to run the code. It will handle the download and installation of the 3rd party libraries required to build and run the code.

Currently there are 3 configurations:
* **CPU only** - `Dockerfile.CPU` and `docker-compose-cpu.yml`. This creates a Docker container for running the simulation purely on the CPU, and without ROS.
* **GPU** (GPU VERSION NOT FUNCTIONAL) - `Dockerfile.GPU` and `docker-compose-gpu.yml`. This creates a Docker container for running the simulation with the GPU, and without ROS.
* **ROS** - `Dockerfile.ROS` and `docker-compose-ros.yml`. This creates a Docker container for running the simulation purely on the CPU with a ROS bridge.

### Linux
First, install the Docker engine (instructions for Ubuntu [here](https://docs.docker.com/engine/install/ubuntu/)).

Then, install the [Docker compose plugin](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

If using a NVIDIA GPU, install the NVIDIA Container Toolkit (instructions for Linux [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)). Be sure to restart the Docker daemon after installing. (NO NEED TO DO THIS AS THE GPU IMPLEMENTATION IS NOT CURRENTLY FUNCTIONAL)

Then, run 

```
xhost +Local:docker
```
in your terminal. This will allow Docker to display graphics using X11 forwarding.

Finally, we are ready to spin up the Docker container. Navigate to the root directory of the repo (where the `Dockerfile.*` and `docker-compose-*.yml` files are). To build the container using Docker compose, run:

```
docker compose -f docker-compose-*.yml up -d --build
```
replacing `docker-compose-*.yml` with one of the Docker compose files provided (i.e. `docker-compose-cpu.yml`, `docker-compose-gpu.yml`, or `docker-compose-ros.yml`).

This may take a while as it downloads and installs the necessary packages and pieces of code. When it has finished building, run:

```
docker ps
```

to ensure that the container is up and running. The output should be something like the following:
![image](https://github.com/user-attachments/assets/dd54e341-ec57-4f8a-a0b6-a82e6ecb475b)
Note the name of the container in the right-most column -- we'll need this to attach to the container to run the code.

To attach to the container, run:

```
docker exec -it sim-*-dev-1 /bin/bash
```

Where `sim-*-dev-1` is the name of the container (this will vary depending on the compose file used, or just use `Tab` to autocomplete the name). Your terminal should switch to a `/workspace` directory, and you're inside the Docker container!

To build the code, navigate to the `/workspace/build` directory. Then run:

```
cmake ..
make -j 8
```

This will create a `Test` binary that we can run with:

```
./Test ../config/config.yaml
```

If all goes well, an Easy3D graphics window should pop up!

To exit out of the container, simply use

```
exit
```
However, the container will still be running at this point. You can `docker ps` to verify this fact. To kill the container, you can use:
```
docker kill sim-dev-1
```

When spinning up and attaching to the container again, you **do not need to rebuild the container** (unless dependencies or the `Dockerfile` have changed). Simply use:
```
docker compose up -d
docker exec -it sim-*-dev-1 /bin/bash
```
### Linux (without Docker)
With Ubuntu 24.04 (and possibly earlier versions of Ubuntu), the repository can also be set up to run without Docker. This process basically just mimics what is done to set up the Docker container.

The dependency installation and environment variables setup is done with the script located in `scripts/install.sh`. The script installs all required external Github repositories and drivers in a directory specified by the user. Example usage:
```
bash scripts/install.sh ../ThirdParty
```
where `../ThirdParty` is the directory where external Github repositories will be cloned and built from source.

Part of `scripts/install.sh` is an environment setup script `scripts/set_env.sh`, which sets a couple of environment variables that are necessary for CMake to find some of the installed repositories. **This does not get added to `~/.bashrc` (though you can manually add it there), so every time the terminal closes and reopens, you will have to rerun `scripts/set_env.sh`.** Like with `scripts/install.sh`, the third-party directory is a required input, and should match the directory given to `scripts/install.sh`. For the example above:
```
bash scripts/set_env.sh ../ThirdParty
```

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


#### ROS
If wanting to use ROS, it is assumed that is already installed on your system. Installation instructions for Ubuntu 24.04 can be found [here](https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html).

Some of the launch files simulatenously launch a Rosbridge server for connecting to Foxglove for visualization. This can be installed with
```
sudo apt-get install ros-jazzy-rosbridge-server -y
```

### Windows
Using WSL, the process should be similar, though there might be some extra stuff to do with the X11 forwarding.


## Changing simulation parameters
This section described how to change simulation parameters to suit your needs.

### Config files
This is the first place you should go to play with simulation parameters. Config files are `.yaml` files that are parsed on simulation launch and used to set parameters of the simulation and all the objects in the simulation. This means that we can change parameters in a config file (e.g. the time step or the size of an object) **without having to rebuild**.

Provided config files are found in the `config/` directory. A commented example config file that should cover most of the options available in config files can be found [here](https://github.com/smtobin/XPBD_Sandbox/blob/1952ffcee7a9623914602a4baf0c7eccfefcebf0/config/example_config.yaml).

### Changing the mesh
The mesh of a simulated deformable object can be changed by simply changing the `filename` parameter. For deformable objects, a volumetric mesh is required (common mesh formats like `.stl` and `.obj` are for surface meshes). Internally, the simulation uses GMSH's mesh format (`.msh`). The simulation is able to take input `.stl` and `.obj` surface mesh files and use GMSH to generate a volumetric `.msh` mesh file from that. A `.msh` file with the same filepath as the input `.stl` or `.obj` file.

### Creating a new derived `Simulation` class (advanced)
If there are specific capabilities not captured by the current simulation types, it is pretty easy to create a new `Simulation` class that provides specific functionality. For an example of how this is done, look at `VirtuosoSimulation`, which extends the base `Simulation` class to add keyboard, mouse, and haptic device control for the Virtuoso robot that is in the simulation.

## Demos
Below describes various off-the-shelf demos that demonstrate some of the simulator capabilities.

### Virtuoso trachea demo
This demo consists of a life-size trachea mesh with a tumor and a Virtuoso robot that can interact with the tissue mesh. The yellow sphere at the tip of the robot represents the grasping radius -- when grasping is toggled to be active, all tissue mesh nodes inside the sphere will be "grasped" and move around with the tip of the robot. When grasping is toggled to be inactive, any grasped mesh nodes are released.

**Running the demo:**
```
./VirtuosoTracheaDemo ../config/demos/virtuoso_trachea/virtuoso_trachea.yaml
```

**Controls:**
Different input types can be used to control the Virtuoso robot. The options are "Keyboard", "Mouse", and "Haptic" (Geomagic Touch). The input type can be changed by changing the `input-device` field in the config file.

For all simulations: press `Alt` to toggle which Virtuoso arm is actively being controlled (only applicable when there is more than one arm). Press `Tab` to switch to the endoscope view.

_Keyboard_: Controls the joint variables directly. `Q/A` = inner tube rotation; `W/S` = outer tube rotation; `E/D` = inner tube translation; `R/F` = outer tube translation. `Space` = toggle grasping.

_Mouse_: Controls the tip of the robot (joint variables computed through inverse kinematics). Hold `Space` and move the mouse to move the active arm tip position parallel to the camera plane. Hold `Space` and scroll the mouse wheel to move the active arm tip position into/out of the camera plane. `Left click` toggles grasping (note: you do not need to hold `Left click` to continually grasp -- one mouse click to toggle on, and another mouse click to toggle off).

_Haptic_: Controls the tip of the robot. Hold the first button and move the input device to move the active arm tip position. Press and hold the second button to grasp (note: you need to continually hold the second button to keep grasping -- hold both buttons to grasp and move at the same time).

**With collisions:**
Collisions between the tissue and the Virtuoso arm can be enabled by running
```
./VirtuosoTracheaDemo ../config/demos/virtuoso_trachea/virtuoso_trachea_collision.yaml
```

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
- **Step 2**: _Generating .ply file with fixed face information._ With the faces you want fixed selected, run Filters --> Quality Measure and Computations --> Per Face Quality Function, and type the expression `fesl*fi` in the user-defined function box. This assigns to each face a quality value of the face index if it is selected, and 0 if not. Then, export the mesh as an ASCII .ply file. Check the box for "Quality", and uncheck the box for "Binary Encoding".
- **Step 3**: _Generating the fixed faces .txt file._ Run the following command to extract only the lines from the .ply file that correspond to the fixed faces:
```
cat <filename>.ply | grep -v " 0 $" | grep "^3 " > fixed_faces.txt
```
Now `fixed_faces.txt` should only have faces that are fixed with the following format:
```
3 <v1> <v2> <v3> <f>
```
This is precisely the format that the simulator is expecting the text file under `fixed-faces-filename` to have.

**Launching with ROS interface:**

From the `ros_workspace` folder (after building the `SimBridge` node - see [ROS interface](#ros-interface)), run
```
ros2 launch launch/sim_bridge_with_rosbridge_server.launch.py config_filename:=../config/demos/fixed_object/fixed_cube.yaml simulation_type:=FixedObjectSimulation
```
There are parameters in the launch file which should be changed, namely `publish_matrices` should be set to `True` (enables publishing of vertices, faces, elements, and stiffness matrix) and `partial_view_pc` should also be set to `True` (enables publishing of partial-view point cloud). Other parameters exist for setting the publish rate and partial-view point cloud parameters (horizontal and vertical FOV, sampling density).

The output topics for the vertices, faces, elements, and stiffness matrix are `/output/vertices_mat_0`, `/output/faces_mat_0`, `/output/elements_mat_0`, and `/output/stiffness_mat_0`. The output topic `/output/mesh_vertices_pc_0` corresponds to the mesh vertices as a point cloud, which is useful for visualization in Foxglove.


## ROS interface
When `docker-compose-ros.yml` is used to build the Docker container, ROS2 Jazzy is installed inside the container. The folder `ros_workspace/` is the ROS workspace folder.

**IMPORTANT:** when using the ROS interface, make sure to `make install` from the `/workspace/build` directory. This is needed so that the ROS node has access to the libraries and headers from the rest of the code.

`sim_bridge` is a provided ROS node that will publish parts of the simulation state over ROS for visualization or integration with other pieces of code.

Two launch files are provided:
* `ros_workspace/launch/sim_bridge.launch.py` - launches the `SimBridge` ROS node by itself.
* `ros_workspace/launch/sim_bridge_with_rosbridge_server.launch.py` - launches the `SimBridge` ROS node and starts a `rosbridge` WebSocket connection on port 9090 (useful for visualizing with Foxglove).

The launch files provide a few launch arguments/parameters:
* Parameter `publish_rate_hz` - the publish rate (in Hz) of the output topics of the `SimBridge` node. Default: 30.0 Hz.
* Launch argument `config_filename` - the absolute path to the config filename to be used to launch the simulation. Default: `/worksapce/config/demos/virtuoso_trachea/virtuoso_trachea.yaml`.
* Launch argument `simulation_type` - the "type" of simulation to be launched. Corresponds to the camel-case class name of the type of simulation to be launched. Default: `VirtuosoTissueGraspingSimulation`. Other options: `GraspingSimulation`, `VirtuosoSimulation`, `Simulation`.

### Example usage

Launching the Virtuoso robot + trachea demo:
```
ros2 launch launch/sim_bridge_with_rosbridge_server.launch.py config_filename:=/workspace/config/demos/virtuoso_trachea/virtuoso_trachea.yaml simulation_type:=VirtuosoTissueGraspingSimulation
```
Launching the simple grasping demo:
```
ros2 launch launch/sim_bridge_with_rosbridge_server.launch.py config_filename:=/workspace/config/demos/simple_grasping/grasping_config.yaml simulation_type:=GraspingSimulation
```
### `sim_bridge` with VirtuosoSimulation
When `simulation_type` is `VirtuosoTissueGraspingSimulation` or `VirtuosoSimulation`, the `sim_bridge` node subscribes to input Virtuoso joint states and relays those to the simulation, and outputs coordinate frames along each Virtuoso arm, as well as the tissue mesh. The list of topics can be found below:

| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/input/arm1_joint_state` | `/ves/left/joint_servo_jp` | Input joint state for the left Virtuoso arm. | N/A | `sensor_msgs/JointState` |   |
| `/input/arm2_joint_state` | `/ves/right/joint_servo_jp`| Input joint state for the right Virtuoso arm. | N/A | `sensor_msgs/JointState` |  |
| `/output/arm1_frames` | `/sim/arm1_frames` | Coordinate frames along left Virtuoso arm. | `/world` | `geometry_msgs/PoseArray` |  |
| `/output/arm2_frames` | `/sim/arm2_frames` | Coordinate frames along backbone of right Virtuoso arm. | `/world` | `geometry_msgs/PoseArray` |  |
| `/output/tissue_mesh` | `/sim/tissue_mesh` | Output surface mesh (vertices, surface faces) of the deformable tissue mesh. | `/world` | `shape_msgs/Mesh` | This message is not timestamped. All vertices are sent, but only surface faces sent.  |
| `/output/tissue_mesh_vertices` | `/sim/tissue_mesh_vertices` | Vertices of the deformable tissue mesh. | `/world` | `sensor_msgs/PointCloud2` | Purely for visualization purposes. |
| `/output/partial_view_pc` | `/sim/partial_view_pc` | A "partial-view" point cloud from the current camera view. | `/world` | `sensor_msgs/PointCloud2` | Generated by casting rays emanating from the current camera position. Use ROS2 parameters `partial_view_pc_hfov`, `partial_view_pc_vfov`, and `partial_view_pc_sample_density` to change horizontal FOV (in degrees), vertical FOV (in degrees), and sample density (rays cast per degree) of the partial view point cloud. Use ROS2 parameter `partial_view_pc` to toggle publishing of the partial-view point cloud on or off (there is some overhead associated with generating the partial-view point cloud). |

### `sim_bridge` in general
In general, the `sim_bridge` node publishes any deformable meshes in the simulation. The list of topics can be found below:
| Topic        | Mapped To | Description | Frame | Type | Notes |
|--------------|-----------|-------------|-------|------|-------|
| `/output/mesh_vertices_<i>` | N/A | Vertices of the ith deformable mesh in the simulation. | `/world` | `sensor_msgs/PointCloud2` | Purely for visualization purposes. |
| `/output/mesh_<i>` | N/A | Output surface mesh (vertices, surface faces) of the ith deformable mesh in the simulation. | `/world` | `shape_msgs/Mesh` | This message is not timestamped. All vertices are sent, but only surface faces sent. |

## Modifying the code
The Docker container is set up to share the repo files outside of the container. That means that you can make edits to files **outside** the container and have those changes be reflected **inside** the container! This is nice because then your code editor does not need to be launched from inside the Docker container.

## Code Structure
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






