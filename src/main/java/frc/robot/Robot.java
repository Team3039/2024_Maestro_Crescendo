// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.subsystems.LightShow.LightShowState;
import frc.robot.subsystems.Shooter.ShooterState;

public class Robot extends TimedRobot {
  private Command autoCommand;

  private static RobotContainer robotContainer;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withStatorCurrentLimit(50);

  public Robot() {
}

  @Override
  public void robotInit() {
    // Pathfinding.setPathfinder(new LocalADStar());
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Blue) {
        } else if(alliance.get() == DriverStation.Alliance.Red){
          RobotContainer.drivetrain.seedFieldRelative(new Pose2d(16, 0, new Rotation2d()));
        }
    }    
    robotContainer = new RobotContainer();

    RobotContainer.drivetrain.getDaqThread().setThreadPriority(99);
    SmartDashboard.putData("Auto Selector", autoChooser);
    for (int module = 0; module < 3; module++) {
      RobotContainer.drivetrain.getModule(module).getDriveMotor().getConfigurator().apply(configs);
  }
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
    RobotContainer.shooter.setState(ShooterState.IDLE);
    RobotContainer.indexer.setState(IndexerState.IDLE);
    RobotContainer.intake.setState(IntakeState.IDLE);
    // RobotContainer.orchestrator.setState(OrchestratorState.SILENT);
  }

  @Override
  public void disabledPeriodic() {
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
    // RobotContainer.wrist.setState(WristState.ALIGN);
  }

  @Override
  public void teleopPeriodic() {
    // RobotContainer.orchestrator.setState(OrchestratorState.MARIOTIME);

    // RobotContainer.drivetrain.redAmpFPH.onlyWhile(() ->
    // RobotContainer.driverPad.getCircleButton());
    // RobotContainer.drivetrain.redTrapClose.onlyWhile(() ->
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
