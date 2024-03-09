// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Wrist.WristState;

public class StartIntakeAuto extends Command {
  /** Creates a new SetIntakeToIntakeMode. */
  public StartIntakeAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.wrist, RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.setState(IntakeState.INTAKING);
    RobotContainer.indexer.setState(IndexerState.INDEXING);
    RobotContainer.wrist.setState(WristState.ALIGN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
