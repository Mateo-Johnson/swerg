# Metrobots 2024/2025 Reefscape Official Robot Code

## Overview
This repository contains the Java code for our FIRST Robotics Competition (FRC) robot. The project is built using the WPILib framework and implements various subsystems and commands to control our robot during both autonomous and teleoperated periods.

## Getting Started

### Prerequisites
- FRC Game Tools (2025 version)
- WPILib (2025 version)
- Visual Studio Code with the WPILib extension
- Git
- Java 17 or newer

### Installation
1. Clone this repository to your local machine: (alternatively use the inbuilt VS Code cloning functionality)
   ```
   git clone https://github.com/metrobots/Metrobots.25.git
   ```
2. Open the project in VS Code using the WPILib extension
3. Build the project to verify everything is working correctly

## Project Structure
- `src/main/java/frc/robot/` - Main robot code
  - `Constants.java` - Contains all constant values used throughout the code
  - `Robot.java` - Main robot class
  - `RobotContainer.java` - Contains subsystem and command initialization
  - `subsystems/` - Subsystem classes for each physical component of the robot
    - `commands/` - Command classes for actions the robot can perform

## Features
- Command-based robot architecture
- Autonomous routines for different starting positions
- Teleoperated controls with driver and operator inputs
- Subsystems include:
  - Drivetrain (Swerve) (MK4i-L3, NEO, Thrifty Encoder)
  - Elevator
  - Intake
  - Climber
  - Vision processing

## Code Deployment
1. Connect to the robot network
2. In VS Code, press Shift+F5 or click the "Deploy Robot Code" button
3. The code will compile and deploy to the RoboRIO

## Contributing
1. Create a new branch for your feature or bug fix
2. Implement and test your changes
3. Submit a pull request for review

## Documentation
- JavaDoc comments are used throughout the codebase
- They contain additional documentation on:
  - Control systems
  - Autonomous routines
  - Hardware mappings
  - Operator controls

## Competition Checklist
- [ ] Verify all motor controller IDs match Constants.java
- [ ] Check battery voltage
- [ ] Test all subsystems individually
- [ ] Run autonomous routines
- [ ] Test driver controls
- [ ] Verify network connectivity and dashboard

## Team
- Lead Programmer: Mateo Johnson
- Programmers: Benny Spanos, Malik Ali Ben Haila
- Mentors: Caleb Caldwell, Andrew Bruening, Nikki Stancampiano

## License
This project is licensed under the MIT License - see the LICENSE file for details.