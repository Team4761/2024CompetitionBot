# 2024CompetitionBot
This is the code for FRC Team 4761's 2024 competition (Crescendo) bot.

Here is the general structure of the code:
robot
- Robot.java
- Main.java
- Constants.java   // Stores constants such as ports, robot dimensions, and control speeds/values
- RobotMap.java    // Stores any physical objects on the robot such as motors, encoders, LEDs, cameras
- subsystems
  - SUBSYSTEM_NAME
    - Subsystem.java
    - Command(s).java
- field
  - AprilTags.java
  - Field.java
- shuffleboard
- auto
  - AutoCommands.java
- simulations

In addition, the libraries we are using are as follows:
- RevRobotics Lib - Swerve
- CTRE Phoenix - Swerve
- PhotonLib - Vision
