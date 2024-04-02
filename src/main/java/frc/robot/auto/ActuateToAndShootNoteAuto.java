// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Vision.VisionState;
import frc.robot.subsystems.Wrist.WristState;
public class ActuateToAndShootNoteAuto extends Command {
  double wristTolerance = 0;
  /** Creates a new ActuateToIdle. */
  public ActuateToAndShootNoteAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
addRequirements(RobotContainer.wrist, RobotContainer.shooter, RobotContainer.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.wrist.setState(WristState.ESTIMATED);
    RobotContainer.shooter.setState(ShooterState.CLOSESHOT);
    RobotContainer.vision.setState(VisionState.ROTATING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (RobotContainer.wrist.isAtSetpoint(.5) && RobotContainer.vision.isAtRotationSetpoint() && RobotContainer.shooter.isAtVelocitySetpoint()){
    RobotContainer.indexer.setState(IndexerState.SHOOTING);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
return RobotContainer.indexer.getState() == IndexerState.SHOOTING;
  }
}
