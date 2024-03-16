// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Elevator.ElevatorState;
// public class ActuateElevatorToIdle extends Command {
//   double elevatorTolerance = 0;
//   /** Creates a new ActuateToIdle. */
//   public ActuateElevatorToIdle(double elevatorTolerance) {
//     // Use addRequirements() here to declare subsystem dependencies.
// addRequirements(RobotContainer.elevator);
// this.elevatorTolerance = elevatorTolerance;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     RobotContainer.elevator.setState(ElevatorState.IDLE);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return  RobotContainer.elevator.isAtSetpoint(2);
//   }
// }
