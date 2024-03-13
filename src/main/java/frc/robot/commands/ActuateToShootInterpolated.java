// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Wrist.WristState;
public class ActuateToShootInterpolated extends Command {
  double wristTolerance = 0;
  /** Creates a new ActuateToIdle. */
  public ActuateToShootInterpolated(double wristTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
addRequirements(RobotContainer.wrist, RobotContainer.shooter);
this.wristTolerance = wristTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.wrist.setState(WristState.INTERPOLATED);
    RobotContainer.shooter.setState(ShooterState.CLOSESHOT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setState(ShooterState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
return false;
  }
}
