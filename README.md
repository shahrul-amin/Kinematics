# 2D SCARA Robot - Kinematics Assignment

A JavaFX application for simulating 2D SCARA robot kinematics with forward and inverse kinematics using Genetic Algorithm.

## Features

- **Forward Kinematics**: Calculate end effector position from joint angles
- **Inverse Kinematics**: Solve for joint angles using Genetic Algorithm
- **Interactive Visualization**: Real-time 2D visualization with animation
- **Multiple Test Cases**: Run predefined test scenarios

## Class Diagram
![Class Diagram](figures/class_diagram.png)

## Requirements

- Java 17 or higher
- JavaFX SDK

## Running the Application

Use the "Run RobotKinematicsApp" launch configuration in VS Code, or run from command line:

```bash
java --module-path lib --add-modules javafx.controls,javafx.graphics -cp bin robotics.RobotKinematicsApp
```

## Launch

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "type": "java",
            "name": "Run RobotKinematicsApp",
            "request": "launch",
            "mainClass": "robotics.RobotKinematicsApp",
            "vmArgs": "--module-path \"REPLACE WITH JAVAFX PATH\" --add-modules javafx.controls,javafx.graphics"
        }
    ]
}
```

## Settings

```json
{
    "java.project.sourcePaths": ["src"],
    "java.project.outputPath": "bin",
    "java.project.referencedLibraries": [
        "lib/**/*.jar"
    ]
}
```
