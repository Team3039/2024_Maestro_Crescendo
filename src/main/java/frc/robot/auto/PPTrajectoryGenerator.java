// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.auto;

// import java.util.HashMap;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.commands.ActuateIntake;
// import frc.robot.commands.ActuateToAmp;
// import frc.robot.commands.WristRoutines.ActuateWristToAlign;
// import frc.robot.commands.WristRoutines.ActuateWristToSetpoint;
// import frc.robot.subsystems.Drive;

// /** Add your docs here. */
// public class PPTrajectoryGenerator {
//     public static Drive drivetrain = RobotContainer.drivetrain;
//     public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

//      public PathPlannerPath redTrapClosePath = PathPlannerPath.fromPathFile("Red Trap Close");
//     public PathPlannerPath redTrapFar = PathPlannerPath.fromPathFile("Red Trap Far");
//     public PathPlannerPath redAmpPath = PathPlannerPath.fromPathFile("Red Amp");

//     public static HashMap<String, Command> eventMap;


  
//     public  PPTrajectoryGenerator() {
//         // eventMap = new HashMap<>();
//         // // eventMap.put("Claw Idle", new SetClawIdleMode());
//         // eventMap.put("Amp Elevate", new ActuateToAmp());
//         eventMap.put("Align Wrist", new ActuateWristToAlign(1));
//         // eventMap.put("Elevator Idle", new SetElevatorIdleMode());
//         // eventMap.put("Elevator Mid Grid", new SetElevatorPositionMode(Constants.Elevator.MID_GRID_SETPOINT));
//         // eventMap.put("Elevator High Grid", new SetElevatorPositionMode(Constants.Elevator.HIGH_GRID_SETPOINT));
        
//     }
// }

