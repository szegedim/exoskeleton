Construct the System: Add two rods. The bottom rod is attached to the ground with a single degree-of-freedom (DOF) joint. The second rod is connected to the first rod with another single DOF joint.

Implement Gravity: Apply a gravitational force that causes each rod to rotate based on its angle.

Add Joint Motors: Equip each joint with a motor. The strength (maximum torque) of each motor should be 150% of the torque required to hold both rods in a horizontal position against the maximum gravitational force acting on them.

Implement Motor Control: Develop a control system for the motors to move the rods towards a vertical orientation, with the system's center of mass directly above the ground joint.

Refine Control: Correct any sign errors in the motor commands. Implement finer-grained control adjustments as the system approaches its equilibrium state to ensure smooth and stable positioning.
