# brc_arm_packages
Packages specific to the BRC robot arm

### brc_arm_description
Contains details about the BRC arm model, specificly in the form of an URDF file. This URDF is generated from Solidworks and contains information such as joint types and limits, link dimensions, collision meshes, etc. It is generated staticly and is mainly used as a reference when launching different nodes from `brc_arm_config`.

### brc_arm_config
*In development*

Contains the bulk of the code for moveit2 with the BRC arm. The `servo_demo.launch.py` file creates an rviz window capable of visualizing and moving the arm based on target positions and motion planning, while the `servo_keyboard_input.cpp` file is used to control the arm in near real time using moveit2_servo features.

Todo:
- Joystick support for servo control
- Joystick support for rviz planning
- Custom code to support toggling between ground pickup and normal mode
- Camera visualtion and support
- Overarching GUI to allow for easier control within one rviz window

### brc_arm_cpp_test
*In development*

### brc_arm_hardware_interface
*In development*
