# Robot Modeling and Control Course Project
## A 6-Axis Robotic Arm Chess Simulation (ROS Noetic Workflow)

### Demo Videos

Simulation against AI
![Demo GIF](videos/demo1.gif)

Robotic Arm Grasping
![Demo GIF](videos/demo2.gif)

Robotic Arm Moving along Cartesian Coordinates
![Demo GIF](videos/demo3.gif)

---

### Quick Start (ROS Noetic + UR5 + Gazebo)

The following process applies to the `ros_noetic_ur5` workspace in this repository (please adjust based on your actual path).

1. **Enter Noetic workspace and compile**
```bash
cd <path-to-repo>/ros_noetic_ur5
catkin_make
source devel/setup.bash
```

2. **Open 4 terminals and source the environment in each**
> Each new terminal needs to be `source`d again, otherwise the ROS package / node may not be found.
```bash
cd <path-to-repo>/ros_noetic_ur5
source devel/setup.bash
```

3. **Terminal 1: Launch Gazebo + MoveIt Demo**
```bash
roslaunch ur5_moveit_config demo_gazebo.launch
```
Wait for all chess pieces to be generated in Gazebo (usually takes a few seconds).

4. **Terminal 2: Publish Chess Piece Pose Information (Note Script Path)**
```bash
python3 <path-to-repo>/ros_noetic_ur5/src/board_new/src/simple_manager.py
```
This script is used to continuously publish chess piece pose information (for the motion planning/grasping node to read).

> If you are already in the workspace root directory (`ros_noetic_ur5`), you can also use a relative path:
```bash
python3 ./src/board_new/src/simple_manager.py
```

5. **Terminal 3: Start Grasping and Input Chess Piece Name to Move**
```bash
rosrun motion_planner move_given_pieces
```
Wait for the prompt:
```text
Enter chess piece name to PICK (e.g., pion1):
```
Enter the name of the piece you want to move (e.g., `benteng4`), and observe the robotic arm performing the **pick and place** action in Gazebo.

6. **Terminal 4: Start Chess GUI (Default Local Two-Player)**
```bash
python3 <path-to-repo>/ros_noetic_ur5/src/ui/chess_gui.py
```
Play chess in the popped-up GUI (default local two-player game). The moves in the GUI will drive the ROS process above (publish/plan/execute), thereby linking the robotic arm to complete the pick and place.

---

### Play against AI (Not Linked with Robotic Arm)

The repository provides `chess_with_ai_no_ros` for playing against an AI engine.
However, due to **conflicts between the AI engine and ROS underlying code**, the **Man-Machine mode** has not yet been integrated into the same workflow as the **robotic arm linkage**; therefore, this mode is only used for pure verification (no ROS).

---

### Tuning Suggestions (Grasping Failure / End Effector "Sticking" to Pieces)

If the following phenomena occur:
- The gripper closes but cannot hold the piece (frequent dropping/clipping)
- The end effector "sticks" to the piece and does not release cleanly
- The piece shakes significantly when moving after grasping

You can try adjusting the `grasp-fix` plugin parameters (related to Gazebo grasping stability):

File Path:
```
ME331ChessBot/ros_noetic_ur5/src/ur5_robot-master/ur5_moveit_config/config/gazebo_ur5.xacro
```
It is recommended to focus on lines around **899–918** for grasp-fix plugin configuration.

Common Parameter Meanings (Naming may vary slightly across versions, refer to actual fields in the file):

| Parameter (Ex. Name) | Function | Adjustment Suggestion |
|---|---|---|
| `update_rate` | Plugin update frequency (Hz), affecting response speed of "grasping/releasing" | Appropriately increase if grasping is unstable; too high may increase computational burden |
| `grip_count_threshold` / `max_grip_count` | Consecutive contact count threshold to determine "firmly grasped" | Piece drops easily: increase appropriately; severe sticking: decrease appropriately or combine with release parameter |
| `release_tolerance` | Threshold/tolerance to allow "release" (usually related to contact/relative speed, etc.) | Not releasing cleanly: increase appropriately; too large may cause premature release |
| `disable_collisions_on_attach` / `disable_collisions` | Whether to temporarily disable collision between gripper and object after grasping (reduce jitter/stuck) | Sticking/jitter can try enabling; but may reduce realistic collision effects |
| `contact_links` / `gripper_link` / `palm_link` | Specify the set of end links used for contact determination/attachment | Ensure configuration is consistent with end link naming in URDF/MoveIt, otherwise it will "not grasp" |

---

### Notes (Placement Logic is Currently Simplified)

The current chess piece **placement position logic has not been fully designed**, and only simple processing has been done (such as placing the grasp point at **y direction +0.2**).
Subsequent modifications/optimizations to the placement strategy can be made in the following files (e.g., calculating target pose based on chessboard grid coordinates, obstacle avoidance, placement rules, etc.):
- `move_given_pieces.cpp`


