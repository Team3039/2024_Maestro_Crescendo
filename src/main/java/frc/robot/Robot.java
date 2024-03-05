// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private static RobotContainer robotContainer;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  Translation2d pose = new Translation2d(12.15, 4.8);

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStar());
    RobotContainer.drivetrain.configurePathPlanner();
   robotContainer = new RobotContainer();
     
    //  PortForwarder.add(1181, "photonvision.local", 1182);
    //  PortForwarder.add(1183, "photonvision.local", 1184);


    RobotContainer.drivetrain.getDaqThread().setThreadPriority(99);
        SmartDashboard.putData("Auto Selector", autoChooser);
 
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    // Pose3d llPose = RobotContainer.vision.photonPoseEstimatorshoot.getReferencePose();
    // RobotContainer.drivetrain.addVisionMeasurement(llPose.toPose2d(), Timer.getFPGATimestamp());
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
  }

  @Override
  public void disabledPeriodic() {
   
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    SignalLogger.start();
    autoCommand = robotContainer.getAutonomousCommand();
        RobotContainer.drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 0 : 180));


    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
			autoCommand.cancel();
		}
    SignalLogger.start();
  }
  

  @Override
  public void teleopPeriodic() {
    // Pose3d llPose = RobotContainer.vision.photonPoseEstimatorshoot.getReferencePose();
    // RobotContainer.drivetrain.addVisionMeasurement(llPose.toPose2d(), kDefaultPeriod, Constants.Vision.kDefaultStdDevs);
    
    //  if (RobotContainer.driverPad.getTriangleButton()){
    //   RobotContainer.intake.setState(IntakeState.INTAKING);
    // };
     if (Pathfinding.isNewPathAvailable()){
      Pathfinding.setStartPosition(RobotContainer.drivetrain.getState().Pose.getTranslation());
      Pathfinding.setGoalPosition(pose);
    }
      // RobotContainer.drivetrain.redAmpFPH.onlyWhile(() -> RobotContainer.driverPad.getCircleButton());
      // RobotContainer.drivetrain.redTrapClose.onlyWhile(() -> RobotContainer.driverPad.getTriangleButton());
    
    // if(RobotContainer.driverPad.getSquareButton()){
    //   RobotContainer.intake.setState(IntakeState.IDLE);
    // }
    
    }   
  

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
