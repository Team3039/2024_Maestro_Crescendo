// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Indexer.IndexerState;
// import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.subsystems.Shooter.ShooterState;
// import frc.robot.subsystems.Vision.VisionState;

// public class NoteTrack extends Command {
//   private double rotation = 0;
//   /** Creates a new NoteTrack. */
//   public NoteTrack() {
//     // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(RobotContainer.drivetrain, RobotContainer.intake, RobotContainer.vision, RobotContainer.indexer); 
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     RobotContainer.vision.setState(VisionState.INTAKING);
//     RobotContainer.intake.setState(IntakeState.INTAKING);
//     RobotContainer.indexer.setState(IndexerState.INDEXING);
//     RobotContainer.vision.getCameraResult(RobotContainer.vision.driveCamera, RobotContainer.vision.resultDriveCamera);
//     RobotContainer.vision.receiveDriveTarget();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.drivetrain.applyRequest(()-> RobotContainer.brake);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
// if (RobotContainer.indexer.hasNote) {
// 			return true;
// 		}
// 		return false;
//   }
// }
