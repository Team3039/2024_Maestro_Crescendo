// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class RotateToTarget extends Command {
  /** Creates a new RotateToTarget. */
  public double error;
  double targetYaw;

  public RotateToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  targetYaw = 0;
    
  }
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(-RobotContainer.driverPad.interpolatedLeftYAxis() * Constants.Drive.MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-RobotContainer.driverPad.interpolatedLeftXAxis() * Constants.Drive.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(2) // Drive counterclockwise with negative X (left)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
