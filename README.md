# 2025 Team Blitz (#2083) FIRST Robotics

## Overview

This project contains the FRC (FIRST Robotics Competition) robot codebase for [Team Blitz](https://teamblitz.net) in Conifer, Colorado. It includes the subsystems, commands, and utilities to control and manage our robots. The project is structured to follow the Command-Based programming paradigm provided by WPILib.

## Competition Overview
This year's competition, [FIRST Dive](https://info.firstinspires.org/first-dive) REEFSCAPE, mimics an underwater reef where the teams must collaborate to remove "algae" and seed new "coral".

## Tools, Languages, and Libraries

### Tools
- **GradleRIO**: A Gradle plugin that simplifies the build and deployment process for FRC robot code.
- **Visual Studio Code**: The recommended IDE for FRC development with WPILib extensions.
- **WPILib**: The core library for FRC robot programming, providing essential classes and functions.

### Languages
- **Java**: The primary programming language used for robot code.

### Libraries
- **WPILib**: Provides the core functionality for robot control and sensor integration.
- **PathplannerLib**: Used for path planning and autonomous routines.
- **AdvantageKit**: Provides additional utilities for logging and debugging.
- **Phoenix**: CTRE library for motor controllers and sensors.
- **REVLib**: Library for REV Robotics hardware.
- **OpenCV**: Used for computer vision processing.

## Project Organization

The project is organized into several directories and files, each serving a specific purpose:

### Root Directory
- **build.gradle**: The main build script for the project.
- **settings.gradle**: Configuration for Gradle, including repository settings.
- **gradlew** and **gradlew.bat**: Gradle wrapper scripts for Unix and Windows.
- **.gitignore**: Specifies files and directories to be ignored by Git.
- **WPILib-License.md**: License information for WPILib.

### Directories
- **.devcontainer/**: Configuration for developing in a containerized environment.
- **.vscode/**: Visual Studio Code settings and configurations.
- **.wpilib/**: WPILib-specific configurations and settings.
- **bin/**: Directory for compiled binaries.
- **build/**: Directory for build outputs.
- **gradle/**: Gradle-specific files and configurations.
- **src/**: Contains the source code for the robot.
  - **main/**: Main source directory.
    - **deploy/**: Files to be deployed to the RoboRIO.
    - **java/**: Java source files.
      - **frc/**: Main package for robot code.
        - **lib/**: Utility classes and libraries.
        - **robot/**: Main robot code, including subsystems and commands.
        - **subsystems/**: Subsystems for different parts of the robot (e.g., drive, elevator, vision).
        - **commands/**: Commands for controlling the robot.
        - **util/**: Utility classes for various purposes.
- **vendordeps/**: Vendor dependency files for external libraries.

### Key Files
- **src/main/java/frc/robot/Robot.java**: The main robot class.
- **src/main/java/frc/robot/Constants.java**: Contains constant values used throughout the robot code.
- **src/main/java/frc/robot/RobotContainer.java**: Configures subsystems, commands, and button bindings.
- **src/main/java/frc/lib/**: Utility classes and libraries used across the project.

## Getting Started

### Prerequisites
- Install [Visual Studio Code](https://code.visualstudio.com/).
- Install [WPILib](https://wpilib.org/).
- Ensure you have Java 17 installed.

### Building and Deploying
1. Open the project in Visual Studio Code.
2. Build the project using the command:
   ```sh
   ./gradlew build
   ```
3. Deploy the code to the RoboRIO using the command:
   ```sh
   ./gradlew deploy
   ```

### Running Simulations
To run simulations, use the following command:
```sh
./gradlew simulateJava
```

## License

This project is licensed under the BSD License. See the [WPILib-License.md](WPILib-License.md) file for details.
