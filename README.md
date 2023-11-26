# Robotics
Use kinematic control on robotic arm to follow and catch a moving sphere

- **Robotics Project**
    - This is a project that aims to control the end-effector of a 6-DOF robotic arm to follow a moving ball and catch it.
    - It consists of two main parts that are explained in detail in the report.
- **Part 1: Kinematic control**
    - This part implements a kinematic controller that adjusts the position and orientation of the end-effector to match the ball's frame.
    - It uses the forward and inverse kinematics, the Jacobian matrix, and the error calculation to generate the input for the joints.
    - It also applies filtering and saturation to the joint velocities and accelerations to ensure safety and stability.
    - It presents the diagrams that show the results of the position, orientation, and joint angles, velocities, and accelerations.
- **Part 2: Catching the ball**
    - This part modifies the desired position of the end-effector to approach the ball and catch it for a few seconds.
    - It uses the same kinematic controller as before, but with a different reference frame for the end-effector.
    - It presents the diagrams that show the results of the position, orientation, and joint angles, velocities, and accelerations.

