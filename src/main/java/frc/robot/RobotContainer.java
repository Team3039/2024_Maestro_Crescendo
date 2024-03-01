// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics.SwerveDriveWheelStates;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.autos.PPTrajectoryGenerator;
// import frc.robot.commands.ActuateIntake;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // 6 meters per second desired top speed
  private static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  public static final  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  // public static final Elevator elevator = new Elevator();
  // public static final Intake intake = new Intake();
  // public static final Vision vision = new Vision();
  // public static final Indexer indexer = new Indexer();
  // public static final Shooter shooter = new Shooter();
  private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
  // private static final SwerveRequest.RobotCentric drives = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate *0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

      
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); // My joystick
  public static final InterpolatedPS4Gamepad operatorPad = new InterpolatedPS4Gamepad(1); // My joystick

    /* Driver Buttons */
    private final JoystickButton driverX = new JoystickButton(driverPad, PS4Controller.Button.kCross.value);
    private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kSquare.value);
    private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Controller.Button.kTriangle.value);
    private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Controller.Button.kCircle.value);
  
    private final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Controller.Button.kL1.value);
    private final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Controller.Button.kR1.value);
    private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
    private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kR2.value);
    private final JoystickButton driverL3 = new JoystickButton(driverPad, PS4Controller.Button.kL3.value);
    private final JoystickButton driverR3 = new JoystickButton(driverPad, PS4Controller.Button.kR3.value);
  
  
    private final JoystickButton driverPadButton = new JoystickButton(driverPad, PS4Controller.Button.kTouchpad.value);
    private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Controller.Button.kPS.value);
  
    private final JoystickButton driverShare = new JoystickButton(driverPad, PS4Controller.Button.kShare.value);
    private final JoystickButton driverOptions = new JoystickButton(driverPad, PS4Controller.Button.kOptions.value);

  

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


   /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public Command runAuto = drivetrain.getAutoPath("Test");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverPad.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverPad.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverPad.getRightX() * MaxAngularRate
            ) // Drive counterclockwise with negative X (left)
        ));

    driverX.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverPad.getLeftY(), -driverPad.getLeftX()))));

    // // reset the field-centric heading on options press
    driverOptions.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // driverCircle.whileTrue(new ActuateIntake());
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  

 
  public RobotContainer() {
    // NamedCommands.registerCommands(PPTrajectoryGenerator.eventMap);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    

  }

  public Command getAutonomousCommand() {
   return runAuto;
   }
}
