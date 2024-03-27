// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbRoutines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb.ClimbState;

public class SetClimbManualOverride extends Command {
  /** Creates a new SetElevatorManualOverride. */
  public SetClimbManualOverride() {
  addRequirements(RobotContainer.climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climb.setState(ClimbState.MANUAL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climb.setState(ClimbState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
