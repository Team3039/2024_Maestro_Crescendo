// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ActuateIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.subsystems.Orchestrator.OrchestratorState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist.WristState;
import frc.util.InterpolatingDouble;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private static RobotContainer robotContainer;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withStatorCurrentLimit(50);
  public Robot() {
}

Pose2d speakerPose = new Pose2d(15.3, 5.50, Rotation2d.fromDegrees(180));

  @Override
  public void robotInit() {
    // Pathfinding.setPathfinder(new LocalADStar());
    
    robotContainer = new RobotContainer();
    RobotContainer.drivetrain.seedFieldRelative(speakerPose);

    // PortForwarder.add(1181, "photonvision.local", 1182);
    // PortForwarder.add(1183, "photonvision.local", 1184);
    RobotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    SmartDashboard.putData("Auto Selector", autoChooser);
    for (int module = 0; module < 3; module++) {
      RobotContainer.drivetrain.getModule(module).getDriveMotor().getConfigurator().apply(configs);
  }
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  //   RobotContainer.vision.getCameraResult(RobotContainer.vision.shootLeftCamera, RobotContainer.vision.resultLeftShooter);
  //   RobotContainer.vision.getCameraResult(RobotContainer.vision.shootRightCamera, RobotContainer.vision.resultRightShooter);
  //   Translation3d rightMultiTagResult = RobotContainer.vision.getMultiTagResult(RobotContainer.vision.shootRightCamera);

  //   if (rightMultiTagResult != null){
  //     Pose2d poseRight = new Pose2d(rightMultiTagResult.toTranslation2d(), Rotation2d.fromDegrees(0));
  //   RobotContainer.drivetrain.addVisionMeasurement(poseRight, kDefaultPeriod, Constants.Vision.kDefaultStdDevs);
  //   System.out.println(poseRight);
  // }
  // Translation3d leftMultiTagResult = RobotContainer.vision.getMultiTagResult(RobotContainer.vision.shootLeftCamera);

  //   if (leftMultiTagResult != null){
  //     Pose2d poseLeft = new Pose2d(leftMultiTagResult.toTranslation2d(), Rotation2d.fromDegrees(0));
  //   RobotContainer.drivetrain.addVisionMeasurement(poseLeft, 
  //   kDefaultPeriod, Constants.Vision.kDefaultStdDevs);
  //   // System.out.println(poseLeft);
  //   }
   
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
    RobotContainer.elevator.setState(ElevatorState.IDLE);
    // RobotContainer.orchestrator.setState(OrchestratorState.SILENT);
  }

  @Override
  public void disabledPeriodic() {
   

    // System.out.println(RobotContainer.drivetrain.odometryIsValid());
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    SignalLogger.start();
    autoCommand = robotContainer.getAutonomousCommand();
    RobotContainer.drivetrain.setOperatorPerspectiveForward(
        Rotation2d.fromDegrees(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 0 : 180));

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
    RobotContainer.wrist.setState(WristState.ALIGN);
  }

  @Override
  public void teleopPeriodic() {
    

    // RobotContainer.orchestrator.setState(OrchestratorState.MARIOTIME);
    // System.out.println(RobotContainer.driverPad.getRightX());

    // RobotContainer.drivetrain.redAmpFPH.onlyWhile(() ->
    // RobotContainer.driverPad.getCircleButton());
    // RobotContainer.drivetrain.redTrapClose.onlyWhile(() ->
    // RobotContainer.driverPad.getTriangleButton());


  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
