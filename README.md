# Team 610 - 2019 Deep Space

## Goals
The goal of this years code was to create a system which had a unit convention (metric - kg, m, s, radians unless otherwise specified), easy to quickly modify the code to add features, easy to debug and ease of use for the drive team and control modifications. This years code was created in a 254 inspired [loop structure](https://github.com/Team610/2019-Public/tree/master/src/main/java/frc/loops) which allowed each subsystem to be self-independent while still all being able to be linked together via the [Subsystem Manager](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/SubsystemManager.java).

## Packages

### Control Systems
This houses any custom control algorithms we use on the robot. For the 2019 season, a [PID](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/controlsystems/PID.java) system was the only custom control type in use. The [PIDF](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/controlsystems/PIDF.java) variant was added for any velocity control on the drivetrain if the NEOs were not up to our standards.

### Loops
Heavily inspired by the 2018 Code from Team 254, the [Loop Manager](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/loops/LoopManager.java). Every periodic method is started via the WPILib Robot init method (robotInit, teleopInit, disabledInit...). The methods are then called automatically in every subsystem for a given loop period.

### Auto
In the 2019 season, we decided on running a driver controlled auto with a Limelight vision system instead of doing autonomous code. With that choice being made, we still run "auto" for our endgame climb onto the HAB level 2 and level 3. The robot was still capable of doing small autos since we did not have enough time to tune and create longer paths. Namely, our robot could drive from the [right HAB level 2](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/auto/modes/RightHabDriveToFrontRightCargo.java) and align itself with the front cargoship to score a hatch. To know the position of the robot at a given time, we calculate the change in position using the change in distance from both encoders of the drive train and the gyro to predict the new position. This is all done in the [Robot State Estimator](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/RobotStateEstimator.java) class.

### Subsystems
Each subsystem's [loop](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/loops/Loop.java) is automatically added to a list of loops to be periodically called by the [loop manager](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/loops/LoopManager.java). Each subsystem is independent of one another and since this season we decided on building an extending arm we needed the [claw](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/Manipulator.java) to be in sync with the [arm](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/Arm.java). The [Subsystem Manager](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/SubsystemManager.java) interfaces any subsystems together.

## Thought Process and Extra

### Controlling the Robot 
In previous years the joystick inputs are written directly when needed in the code. This means we used to have if statements scattered everywhere in the codebase and changing from one button to another was a pain. This year we put every control input into its own class, the [controls](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/Controls.java) class. Similar to having a constants class for port numbers, the controls class allows for easy changing of button layouts without searching the code for multiple references.

### Controlling the Arm
Controlling the arm this year had two focuses in mind, speed and position. We wanted the arm to move as quickly as possible from any position on the robot to the setpoint without leaving the robot's bumpers. The easiest way to accomplish this was to rotate to an angle, then extended, all while the claw has calculated its angle and is rotating to its setpoint. After finishing the algorithm to use for the [Arm's control](https://github.com/Team610/2019-Public/blob/bad5caaa3937a124b5f7b904b144c7c542568974/src/main/java/frc/robot/subsystems/Arm.java#L114), we needed to figure out a simpler and more intuitive way to create presets. Normally our team finds the ticks that the roborio displays and use a PID system to achieve the value. This year we converted all our ticks to a cartesian coordinate system based on radians and meters. This allowed us to create presets that were very close to accurate based on real-life measurements the first time. After figuring out the arm logic we create multiple control methods for the claw. We initially thought that if the claw was always facing the rocket we could extend and rotate at the same time. This worked great and was super fast when going from intaking or center preset to any rocket level. The trouble came from when we went from a high preset to an intaking preset (the arm would run full speed into the ground). After realizing we did not have enough time to invest in making that form of control, we decided to calculate the angle of the claw from where the arm will be after it finishing position locking. All this math is fairly easy (just some trig) with our units now being converted to radians and meters.

### Vision
We knew from the very start of the season that [vision](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/Limelight.java) was going to be a huge part of the game. There are many scoring areas where it is hard to see the robot depending on the driver-station location and where the robot is. We quickly swapped from a custom camera array setup to using two limelights both facing opposite directions to enhance our double-sided scoring capabilities. We activate the vision on a toggle and chose which limelight to turn on based on the given angle that the arm is at. Any angle greater than PI/2 turns one limelight on and any angle less than PI/2 turns the other one on. The limelights control the turning offset of the drive while our driver controls the throttle allowing any on-the-fly speed adjustments to be controlled. Once our driver ejects or intakes the hatch the vision systems turn off and full robot control is regained. 

### Presets
We knew instantly after deciding on building an extending arm that [presets](https://github.com/Team610/2019-Public/blob/bad5caaa3937a124b5f7b904b144c7c542568974/src/main/java/frc/robot/Constants.java#L150) were going to be a big deal and we needed a way to create modularly and while being intuitive. With these goals in mind, our presets were designed with a flag type process. Each button our operator presses adds a flag to the preset string. Once a flag is finished the robot automatically sets all the wanted angles and extensions to move towards. The presets contain [complete states](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/subsystems/ArmAndManipulatorState.java) for the arm position and the claw calculation type. Once selected the Subsystem Manager takes over and combines both the claw and the arm together to reach preset.

### HAB Climb
Our HAB climbing is completely automated allowing us to get a consistent HAB rp at the end of every match by climbing [HAB level 3](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/auto/modes/HangLvl3.java) or the 6 points from [HAB level 2](https://github.com/Team610/2019-Public/blob/master/src/main/java/frc/robot/auto/modes/HangLvl2.java). The process to climb is: driving into the HAB to align the robot, pushing with the stilts at full power while aligning the robot's pitch to be parallel to the floor using a PidgeonIMU gyro and the arm, locking the stilts while driving in and lastly retracting the stilts.
