// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Wrist.WristState;

public class SpinUpSubwoofer extends Command {
  /** Sets Robot To Shoot Against Speaker **/
  public SpinUpSubwoofer() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // RobotContainer.wrist.setState(WristState.ALIGN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.shooter.setState(ShooterState.CLOSESHOT);
    RobotContainer.wrist.setState(WristState.CLOSESHOT);
    // RobotContainer.wrist.setState(WristState.INTERPOLATED);
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
