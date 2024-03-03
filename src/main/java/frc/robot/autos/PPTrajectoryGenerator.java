// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos;

// import java.util.HashMap;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// /** Add your docs here. */
// public class PPTrajectoryGenerator {
//     public static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
//     public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

//     public static HashMap<String, Command> eventMap;

//     public static AutoBuilder autoBuilder = drivetrain.getAutoPath("null");

//     public static PathPlannerTrajectory getBottomPathTwoPiece() {
//         return PathPlannerAuto.loadPath("Bottom 2 Piece YP", 
//             new PathConstraints(4.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
//     }
//     public  PPTrajectoryGenerator() {
//         eventMap = new HashMap<>();
//         // eventMap.put("Claw Idle", new SetClawIdleMode());
//         // eventMap.put("Intake", new SetClawIntakeMode());
//         // eventMap.put("Elevator Idle", new SetElevatorIdleMode());
//         // eventMap.put("Elevator Mid Grid", new SetElevatorPositionMode(Constants.Elevator.MID_GRID_SETPOINT));
//         // eventMap.put("Elevator High Grid", new SetElevatorPositionMode(Constants.Elevator.HIGH_GRID_SETPOINT));
//         // eventMap.put("Wrist Idle", new SetWristIdleMode());
//         // eventMap.put("Wrist Position", new SetWristPositionMode(0));

//         ;
//     }

//     public static AutoBuilder getAutoBuilder() {
//         return autoBuilder;
//     }  
// }

