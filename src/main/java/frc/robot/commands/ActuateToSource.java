// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ElevatorRoutines.ActuateElevatorToSetpoint;
import frc.robot.commands.WristRoutines.ActuateWristToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ActuateToSource extends SequentialCommandGroup {
  /** Creates a new ActuateToAmp. */
  public ActuateToSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ActuateElevatorToSetpoint(Constants.Elevator.ELEVATOR_TO_SOURCE, 2),
        new ActuateWristToSetpoint(Constants.Wrist.WRIST_TO_SOURCE, 2),
        new IntakeSource());
  }
}
