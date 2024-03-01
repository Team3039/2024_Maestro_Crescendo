// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autos;

// import java.util.HashMap;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Drive;

// /** Add your docs here. */
// public class PPTrajectoryGenerator {
//     public static Drive swerve = RobotContainer.drive;
//     public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

//     public static HashMap<String, Command> eventMap;

//     public static SwerveAutoBuilder autoBuilder;

//     public static PathPlannerTrajectory getBottomPathTwoPiece() {
//         return PathPlanner.loadPath("Bottom 2 Piece YP", 
//             new PathConstraints(4.0, 4.0));
//     }
    
//     public static PathPlannerTrajectory getBottomPathDriveOut() {
//         return PathPlanner.loadPath("Bottom Path Drive Out", 
//             new PathConstraints(4.0, 4.0));
//     }
    
//     public static PathPlannerTrajectory getTopPathTwoPiece() {
//         return PathPlanner.loadPath("Top 2 Piece YP", 
//             new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getTopPath3rdPiece() {
//         return PathPlanner.loadPath("Top 3rd Piece", 
//             new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getTopPathDriveOut() {
//         return PathPlanner.loadPath("Top Path Drive Out", 
//             new PathConstraints(4.0, 4.0));      
//     }

//     public static PathPlannerTrajectory getDriveOut() {
//         return PathPlanner.loadPath("Drive Out", 
//             new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getChargeStationTopLPath() {
//         return PathPlanner.loadPath("Charge Station Top L Path", 
//             new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getChargeStationBottomLPath() {
//         return PathPlanner.loadPath("Charge Station Bottom L Path", 
//             new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getBalanceAfterTopTwoPiece() {
//         return PathPlanner.loadPath("Balance After Top Two Piece", 
//         new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverDriveOverWireCover() {
//         return PathPlanner.loadPath("Wire Cover Drive Over Wire Cover", 
//         new PathConstraints(1.50, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverGrabFirstPiece() {
//         return PathPlanner.loadPath("Wire Cover Grab First Piece", 
//         new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverGrabSecondPiece() {
//         return PathPlanner.loadPath("Wire Cover Grab Second Piece", 
//         new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverDriveToDispensePiece() {
//         return PathPlanner.loadPath("Wire Cover Drive To Dispense Piece", 
//         new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverDriveBackToWireCover() {
//         return PathPlanner.loadPath("Wire Cover Drive Back To Wire Cover", 
//         new PathConstraints(4.0, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverDriveBackOverWireCover() {
//         return PathPlanner.loadPath("Wire Cover Drive Back Over Wire Cover", 
//         new PathConstraints(1.5, 4.0));
//     }

//     public static PathPlannerTrajectory getWireCoverEndRotate() {
//         return PathPlanner.loadPath("Wire Cover End Rotate", 
//         new PathConstraints(3, 4.0));
//     }

//     public static PathPlannerTrajectory getMidTaxiGrabPiece() {
//         return PathPlanner.loadPath("Mid Taxi Grab Piece", 
//         new PathConstraints(3, 4.0));
//     }
//     public PPTrajectoryGenerator() {
//         eventMap = new HashMap<>();
//         // eventMap.put("Claw Idle", new SetClawIdleMode());
//         // eventMap.put("Intake", new SetClawIntakeMode());
//         // eventMap.put("Elevator Idle", new SetElevatorIdleMode());
//         // eventMap.put("Elevator Mid Grid", new SetElevatorPositionMode(Constants.Elevator.MID_GRID_SETPOINT));
//         // eventMap.put("Elevator High Grid", new SetElevatorPositionMode(Constants.Elevator.HIGH_GRID_SETPOINT));
//         // eventMap.put("Wrist Idle", new SetWristIdleMode());
//         // eventMap.put("Wrist Position", new SetWristPositionMode(0));

//         autoBuilder = new SwerveAutoBuilder(
//             swerve::getPose,
//             swerve::resetOdometry,
//             Constants.Swerve.SWERVE_KINEMATICS,
//             new PIDConstants(1.0, 0.0, 0.0),
//             new PIDConstants(1.0, 0.0, 0.0),
//             swerve::setModuleStates,
//             eventMap,
//             true,
//             swerve
//             );
//     }

//     public static SwerveAutoBuilder getAutoBuilder() {
//         return autoBuilder;
//     }  
// }

