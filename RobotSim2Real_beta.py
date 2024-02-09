import pybullet as p
import pybullet_data as pd
import numpy as np
import time
import math
import os
import sys
#from xarm.wrapper import XArmAPI

p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
# p.setGravity(0, 0, -9.8) # TEST this in real, is probably default

useFixedBase = True
flags = p.URDF_USE_SELF_COLLISION

# Set camera settings
camera_distance = 1.75
p.resetDebugVisualizerCamera(camera_distance, 23, -34, [0, 0, 0])
manual_control_enabled = False  # Flag for manual control mode
is_capture_image_active = False # Flag for sysnthetic data
previous_capture_image_state = 0
initial_state = True

# Load environment and robot
ip = '192.168.1.242'  # xArm's IP address
table_pos = [0, 0, -0.625]
table = p.loadURDF("table/table.urdf", table_pos, flags=flags, useFixedBase=useFixedBase)
xarm = p.loadURDF("meshes/xarm7/xarm7_robot.urdf", flags=flags, useFixedBase=useFixedBase)

initial_state_id = p.saveState() # Store initial state as home position
num_joints = p.getNumJoints(xarm)

# Initialize storage for recorded state IDs
recorded_state_ids = []
num_joints = p.getNumJoints(xarm)
jointIds = []
paramIds = []
jointNames = []

def paintRobot(): # Define colors for each link (RGBA format) 
    colors = [
        [1, 0, 0, 1],    # Red
        [0, 1, 0, 1],    # Green
        [0, 0, 1, 1],    # Blue
        [1, 1, 0, 1],    # Yellow
        [1, 0, 1, 1],    # Magenta
        [0, 1, 1, 1],    # Cyan
        [0.5, 0.5, 0.5, 1],  # Gray
        [0.6, 0.6, 0.6, 1],  # Gray
        # ... add more colors for additional links
    ]
    # Assign a color to each link
    for link_index in range(p.getNumJoints(xarm)):
        # Ensure we have enough colors defined
        if link_index < len(colors):
            color = colors[link_index]
            p.changeVisualShape(xarm, linkIndex=link_index, rgbaColor=color)
        else:
            print(f"No color defined for link index {link_index}")

#Captures a synthetic image from the simulation if activated.
def capture_image(activate):
    if activate:
        # Define camera parameters
        camera_position = [1, 1, 1]  # Camera position in world coordinates
        camera_target = [0, 0, 0]  # The point in world space the camera looks at
        camera_up_vector = [0, 0, 1]  # Up vector in world space to define the camera's vertical
        fov = 60  # Field of view
        aspect = 1  # Aspect ratio
        near_val = 0.1  # Near clipping plane
        far_val = 100  # Far clipping plane
        image_width = 320  # Image width in pixels
        image_height = 240  # Image height in pixels
        capture_frequency = 10  # Capture an image every 10 steps
        renderer = p.ER_BULLET_HARDWARE_OPENGL  # or ER_TINY_RENDERER for CPU-based rendering
    
        # Compute view and projection matrices
        view_matrix = p.computeViewMatrix(camera_position, camera_target, camera_up_vector)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near_val, far_val)
    
        # Capture the image
        image = p.getCameraImage(image_width, 
                                 image_height, 
                                 view_matrix, 
                                 projection_matrix, 
                                 shadow=True, 
                                 renderer=renderer)
        return image
    else:
        return None

# Create sliders for joint control in degrees
for j in range(num_joints):
    info = p.getJointInfo(xarm, j)
    jointName = info[1]
    jointType = info[2]
    if jointType in (p.JOINT_PRISMATIC, p.JOINT_REVOLUTE):
        jointIds.append(j)
        displayName = " " + jointName.decode("utf-8")  
        jointNames.append(jointName.decode("utf-8"))
        paramIds.append(p.addUserDebugParameter(displayName, -180, 180, 0))

def update_sliders_to_current_joint_angles(robot_id, joint_names):
    global jointIds, paramIds
    # Clear the paramIds list
    paramIds = []
    # Create new sliders
    for i, jointName in enumerate(joint_names):
        jointState = p.getJointState(robot_id, jointIds[i])
        jointAngle = math.degrees(jointState[0])
        displayName = " " + jointName
        new_param_id = p.addUserDebugParameter(displayName, -180, 180, jointAngle)
        paramIds.append(new_param_id)

# Add a non-functional slider as an instruction label
instruction_id = p.addUserDebugParameter(" Control Key: 1=Mouse; 2=Sliders On/Off", 0, 1, 1)
instruction_text = "INSTRUCTIONS: Use [R] to record, [E] to execute, [X] to reset."

def print_joint_angles(robot_id, joint_ids, joint_names):
    for joint_id, joint_name in zip(joint_ids, joint_names):
        jointState = p.getJointState(robot_id, joint_id)
        jointAngle = math.degrees(jointState[0])
        print(f"  {joint_name}: {jointAngle:.2f} degrees")
    print() #

def revert_to_mouse_control(robot_id, joint_ids):
    for joint_id in joint_ids:
        jointState = p.getJointState(robot_id, joint_id)
        currentPos = jointState[0]  # Get current joint position
        # Set the joint to hold its current position
        p.setJointMotorControl2(robot_id, joint_id, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, targetPosition=currentPos)#, force=0)

def disable_manual_control(robot_id, joint_ids):
    for joint_id in joint_ids:
        jointState = p.getJointState(robot_id, joint_id)
        currentPos = jointState[0]  # Get current joint position
        # Set the joints to velocity control with zero velocity and the current position
        p.setJointMotorControl2(robot_id, joint_id, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=65)

def showSynthetics(enable):
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, enable)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW , enable)

# Add buttons to the GUI
def add_buttons():
    global capture_button_id, execute_button_id, reset_button_id, capture_image_button_id
    capture_button_id = p.addUserDebugParameter("  Record Pose  ", 1, 0, 0)  # Button for recording
    execute_button_id = p.addUserDebugParameter("  Execute Pose Sequence  ", 1, 0, 0)  # Button for execution
    reset_button_id = p.addUserDebugParameter("  Reset Recorded Sequence  ", 1, 0, 0)  # Button for resetting
    capture_image_button_id = p.addUserDebugParameter(" Capture Synthetic Image", 1, 0, 0) # Button for synthetic data


# Initialize variables to track the last button press count
last_capture_button_press = 0
last_execute_button_press = 0
last_reset_button_press = 0

# Initial setup: create buttons
add_buttons()

def record_current_state():
    state_id = p.saveState()
    recorded_state_ids.append(state_id)
    print("Recorded current state.")

def execute_recorded_states():
    print("Executing recorded sequences.")
    for state_id in recorded_state_ids:
        p.restoreState(stateId=state_id)
        time.sleep(1)  # Adjust time as needed for your simulation
    else: print("No recorded states to execute.")

def reset_recorded_states():
    global recorded_state_ids
    for state_id in recorded_state_ids:
        p.removeState(state_id)
    recorded_state_ids = []  # Clear the list after removing states
    print("Cleared all recorded state sequences.")
temp = 0

def viewSynthetics():
    global is_capture_image_active, previous_capture_image_state, temp
    capture_image_state = p.readUserDebugParameter(capture_image_button_id)

    # If the button state has increased (indicating a press), toggle the capture state
    if capture_image_state > previous_capture_image_state:
        is_capture_image_active = not is_capture_image_active
        showSynthetics(1)
        print("Synthetic data capture: Active.")
    previous_capture_image_state = capture_image_state # Update the previous button state
    if is_capture_image_active: # If image capture is active, capture an image
        image = capture_image(True); temp = 1
    else: # Disable capturing
        if temp == 1:
            showSynthetics(0); temp = 0

while True:
    p.stepSimulation()
    if initial_state: # After setting up robot and environment
        initial_state_id = p.saveState()
        initial_state = False
    #paintRobot() # Assign Colours to the robot links 
    keys = p.getKeyboardEvents()
    mouse_events = p.getMouseEvents()

    # Read the current button press count
    current_capture_press = p.readUserDebugParameter(capture_button_id)
    current_execute_press = p.readUserDebugParameter(execute_button_id)
    current_reset_press = p.readUserDebugParameter(reset_button_id)
    viewSynthetics()

    # Check if the capture button was pressed since the last iteration
    if current_capture_press > last_capture_button_press:
        record_current_state() # Action to record current state

    # Check if the execute button was pressed since the last iteration
    if current_execute_press > last_execute_button_press:
        execute_recorded_states() # Action to execute recorded states
 
    # Check if the reset button was pressed since the last iteration
    if current_reset_press > last_reset_button_press:
        reset_recorded_states() # Action to reset recorded states

    # Update the last button press count
    last_capture_button_press = current_capture_press
    last_execute_button_press = current_execute_press
    last_reset_button_press = current_reset_press

    for event in mouse_events: # Check for mouse release and print joint angles
        eventType, mousePosX, mousePosY, buttonIndex, buttonState = event
        # Check for left mouse button release event
        if eventType == 2 and buttonIndex == 0 and buttonState & p.KEY_WAS_RELEASED:
            print_joint_angles(xarm, jointIds, jointNames)

    if ord('2') in keys and keys[ord('2')] & p.KEY_WAS_TRIGGERED: # Toggle manual control with '2' key
        if manual_control_enabled: # Disable manual control
            manual_control_enabled = False
            disable_manual_control(xarm, jointIds)

        else: # Enable manual control 
            manual_control_enabled = True
            p.removeAllUserParameters() # Clear all user parameters (sliders and buttons)
            # Update sliders to match current joint angles
            update_sliders_to_current_joint_angles(xarm, jointNames)
            add_buttons()
        print(" Button Sliders control mode:", "Enabled" if manual_control_enabled else "Disabled")

    if ord('1') in keys and keys[ord('1')] & p.KEY_WAS_TRIGGERED: # Revert to mouse control with '1' key
        manual_control_enabled = False
        print(" Home position with mouse control: Enabled")
        p.restoreState(stateId=initial_state_id)
        for joint_id in jointIds: # Loop through all joints and set the default force
            # Readjust POSITION_CONTROL + VELOCITY_CONTROL
            p.setJointMotorControl2(bodyUniqueId=xarm, 
                                    jointIndex=joint_id, 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocity=0, 
                                    force=60)
    
    if manual_control_enabled: # Read and apply slider values to joints if manual control is enabled
        for i, jointId in enumerate(jointIds):
            targetPosDegrees = p.readUserDebugParameter(paramIds[i])
            targetPosRadians = math.radians(targetPosDegrees)
            p.setJointMotorControl2(xarm, jointId, p.POSITION_CONTROL, targetPosRadians)

    activate_image_capture = False #True
    image = capture_image(activate_image_capture)
    time.sleep(1./240.)