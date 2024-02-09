### GitHub Repository Description for the xARM-7 Simulation Project

**Project Title:** RobSim2Real

**Description:**

This project presents a comprehensive simulation platform developed to facilitate rapid prototyping, testing, and motion transfer between the virtual and physical xARM-7 robotic arm. Utilizing the PyBullet physics engine and the xARM SDK, the platform offers detailed and realistic digital representations of the xARM-7, enabling intuitive interaction and direct control through a user-friendly interface.
Control keys:
1. press once 2-key to switch control to botton sliders (right corner GUI)
2. press again 2-key to switch back to mouse cursor control
3. 1-key to return the robotic arm to Home pose (initial pose)
4. g-key to show-hide GUI menu (buttons + synthetic data + top/bottom info bars)
5. s-key to show/hide shadows
6. w-key to show/hide mesh view
7. v-key to shoe/hide solid bodies view
8. Buttons:  A. Record/store robotic arm current pose (you can record any sequence of poses)
             B. Execute the pose sequence (stored)
             C. Reset stored sequence
             D. Show/hide synthetic data (depth and segmentation vision)
   
**Features:**

- **Realistic Physics Simulation:** Powered by PyBullet, the environment accurately models the dynamics of the xARM-7, including rigid and soft body interactions, for precise motion replication.
- **Intuitive Control System:** Users can manipulate the robotic arm in the simulation using mouse inputs or sliders for joint control, offering a seamless transition between manual adjustments and automated control mechanisms.
- **Pose Recording and Playback:** This feature allows the capture and playback of specific arm poses, facilitating the demonstration of complex tasks and the validation of control strategies.
- **Synthetic Data Generation:** The platform can generate synthetic images from the simulation's camera view, useful for computer vision applications or debugging purposes.
- **Safety Features:** Incorporates safety protocols such as self-collision detection and error handling to ensure a smooth transition from simulation to real-world application.

**Light Version Distribution:**

Accompanying this repository, we provide a light version of the simulation environment as part of the project report. This version focuses on the simulation aspects without requiring a connection to the physical robotic arm, making it suitable for broader educational and research purposes.

**Fully Developed Version for Internal Use:**

The complete version of the xARM-7 Simulation Environment, which includes features for direct interaction with the physical xARM-7 robotic arm, is documented in this GitHub repository. Due to the proprietary nature of certain components access to the full version is restricted for internal use.

**Contributions and Feedback:**

We welcome contributions and feedback from the community to improve the simulation environment further. If you have suggestions, please feel free to open an issue or submit a pull request.

**License:**

N/A.

**Acknowledgments:**

Special thanks to the contributors, the PyBullet team for their robust physics engine, and the xARM team for their support and the comprehensive SDK that made this project possible.

---

**NOTE:** The light version of the simulation, distributed with this report, is designed for wide accessibility. The fully developed version, detailed on GitHub, is intended for internal use, aligning with our commitment to secure and responsible robotics research and development.
