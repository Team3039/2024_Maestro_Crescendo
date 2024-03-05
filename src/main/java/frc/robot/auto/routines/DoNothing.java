// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DoNothing extends SequentialCommandGroup {

  public DoNothing() {
    addCommands(
      new InstantCommand(() -> System.out.println("Do nothing poggers"))
    );
  }
}
