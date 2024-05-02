
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.ActuateToAndFeedNote;
import frc.robot.auto.ActuateToAndShootNoteAuto;
import frc.robot.auto.ActuateToShootCalculatedAuto;
import frc.robot.auto.ActuateToShootFastAuto;
import frc.robot.auto.ActuateWristToCloseShotAuto;
import frc.robot.auto.IndexerStartShootAuto;
import frc.robot.auto.IndexerStopShootAuto;
import frc.robot.auto.RotateToSpeakerAuto;
import frc.robot.auto.SpinUpSubwooferAuto;
import frc.robot.auto.StartIntakeAuto;
import frc.robot.auto.StopIntakeAuto;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.ActuateRelease;
import frc.robot.commands.ActuateShooterToCLIMB;
import frc.robot.commands.ActuateToAndShootNote;
import frc.robot.commands.FeedFromMiddle;
import frc.robot.commands.IndexerToShoot;
import frc.robot.commands.ShootAMP;
import frc.robot.commands.SpinUpSubwoofer;
import frc.robot.commands.ClimbRoutines.SetClimbManualOverride;
import frc.robot.commands.WristRoutines.ActuateWristToAlign;
import frc.robot.commands.WristRoutines.ActuateWristToSetpoint;
import frc.robot.commands.WristRoutines.SetWristManualOverride;
import frc.robot.controllers.InterpolatedPS4Gamepad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightShow;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  public static final Intake intake = new Intake();
  public static final Vision vision = new Vision();
  public static final Wrist wrist = new Wrist();
  public static final Indexer indexer = new Indexer();
  public static final Shooter shooter = new Shooter();
  public static final Climb climb = new Climb();
  public static final Drive drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public static final LightShow lightShow = new LightShow();

  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Drive.MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 5% Deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /*
   * InterpolatedPS4GamePad Treats Axis Inputs Exponentially Instead Of Linearly
   */
  // public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(3);
  public static final InterpolatedPS4Gamepad driverPad = new InterpolatedPS4Gamepad(0); // Pilot Joystick
  public static final InterpolatedPS4Gamepad operatorPad = new InterpolatedPS4Gamepad(1); // Co-Pilot Joystick
  // public static final InterpolatedPS4Gamepad testPad = new InterpolatedPS4Gamepad(2); // Testing Joystick
  // driver Xbox 

//    private final JoystickButton xboxdriverX = new JoystickButton(driverPad, XboxController.Button.kCross.value);
//    private final JoystickButton xboxdriverSquare = new JoystickButton(driverPad, XboxController.Button.kSquare.value);
//    private final JoystickButton xobxdriverTriangle = new JoystickButton(driverPad, XboxController.Button.kTriangle.value);
//    private final JoystickButton xboxdriverCircle = new JoystickButton(driverPad, XboxController.Button.kCircle.value);

//   private final JoystickButton xboxdriverL1 = new JoystickButton(driverPad, XboxController.Button.kL1.value);
//   private final JoystickButton xboxdriverR1 = new JoystickButton(driverPad, XboxController.Button.kL2.value);
//   private final JoystickButton xboxdriverL2 = new JoystickButton(driverPad, XboxController.Button.kL2.value);
//   private final JoystickButton xboxdriverR2 = new JoystickButton(driverPad, XboxController.Button.kR2.value);
//   private final JoystickButton xboxdriverL3 = new JoystickButton(driverPad, XboxController.Button.kL3.value);
//   private final JoystickButton xboxdriverR3 = new JoystickButton(driverPad, XboxController.Button.kR3.value);

//  private final JoystickButton xboxdriverPadButton = new JoystickButton(driverPad, XboxController.Button.kTouchpad.value);
//  private final JoystickButton xboxdriverStart = new JoystickButton(driverPad, XboxController.Button.kPS.value);

//  private final JoystickButton xboxdriverShare = new JoystickButton(driverPad, XboxController.Button.kShare.value);
//  private final JoystickButton xboxdriverOptions = new JoystickButton(driverPad, XboxController.Button.kOptions.value);




  // Driver Ps4 
   private final JoystickButton driverX = new JoystickButton(driverPad, PS4Controller.Button.kCross.value);
   private final JoystickButton driverSquare = new JoystickButton(driverPad, PS4Controller.Button.kSquare.value);
   private final JoystickButton driverTriangle = new JoystickButton(driverPad, PS4Controller.Button.kTriangle.value);
   private final JoystickButton driverCircle = new JoystickButton(driverPad, PS4Controller.Button.kCircle.value);

  private final JoystickButton driverL1 = new JoystickButton(driverPad, PS4Controller.Button.kL1.value);
  private final JoystickButton driverR1 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
  private final JoystickButton driverL2 = new JoystickButton(driverPad, PS4Controller.Button.kL2.value);
  private final JoystickButton driverR2 = new JoystickButton(driverPad, PS4Controller.Button.kR2.value);
  private final JoystickButton driverL3 = new JoystickButton(driverPad, PS4Controller.Button.kL3.value);
  private final JoystickButton driverR3 = new JoystickButton(driverPad, PS4Controller.Button.kR3.value);

 private final JoystickButton driverPadButton = new JoystickButton(driverPad, PS4Controller.Button.kTouchpad.value);
 private final JoystickButton driverStart = new JoystickButton(driverPad, PS4Controller.Button.kPS.value);

 private final JoystickButton driverShare = new JoystickButton(driverPad, PS4Controller.Button.kShare.value);
 private final JoystickButton driverOptions = new JoystickButton(driverPad, PS4Controller.Button.kOptions.value);

  /* Operator Buttons */
  private final JoystickButton operatorX = new JoystickButton(operatorPad, PS4Controller.Button.kCross.value);
  private final JoystickButton operatorSquare = new JoystickButton(operatorPad, PS4Controller.Button.kSquare.value);
  private final JoystickButton operatorTriangle = new JoystickButton(operatorPad, PS4Controller.Button.kTriangle.value);
  private final JoystickButton operatorCircle = new JoystickButton(operatorPad, PS4Controller.Button.kCircle.value);

  private final JoystickButton operatorL1 = new JoystickButton(operatorPad, PS4Controller.Button.kL1.value);
  private final JoystickButton operatorR1 = new JoystickButton(operatorPad, PS4Controller.Button.kR1.value);

  private final JoystickButton operatorL2 = new JoystickButton(operatorPad, PS4Controller.Button.kL2.value);
  private final JoystickButton operatorR2 = new JoystickButton(operatorPad, PS4Controller.Button.kR2.value);
  private final JoystickButton operatorR3 = new JoystickButton(operatorPad, PS4Controller.Button.kR3.value);

  private final JoystickButton operatorPadButton = new JoystickButton(operatorPad,
      PS4Controller.Button.kTouchpad.value);
  private final JoystickButton operatorStart = new JoystickButton(operatorPad, PS4Controller.Button.kPS.value);

  private final JoystickButton operatorShare = new JoystickButton(operatorPad, PS4Controller.Button.kShare.value);
  private final JoystickButton operatorOptions = new JoystickButton(operatorPad, PS4Controller.Button.kOptions.value);

//   /* Operator Buttons */
//   private final JoystickButton testX = new JoystickButton(testPad, PS4Controller.Button.kCross.value);
//   private final JoystickButton testSquare = new JoystickButton(testPad, PS4Controller.Button.kSquare.value);
//   private final JoystickButton testTriangle = new JoystickButton(testPad, PS4Controller.Button.kTriangle.value);
//   private final JoystickButton testCircle = new JoystickButton(testPad, PS4Controller.Button.kCircle.value);

//   private final JoystickButton testL1 = new JoystickButton(testPad, PS4Controller.Button.kL1.value);
//   private final JoystickButton testR1 = new JoystickButton(testPad, PS4Controller.Button.kR1.value);

  // private final JoystickButton testL2 = new JoystickButton(testPad, PS4Controller.Button.kL2.value);
  // private final JoystickButton testR2 = new JoystickButton(testPad, PS4Controller.Button.kR2.value);
//   private final JoystickButton testR3 = new JoystickButton(testPad, PS4Controller.Button.kR3.value);

//   private final JoystickButton testPadButton = new JoystickButton(testPad, PS4Controller.Button.kTouchpad.value);
//   private final JoystickButton testStart = new JoystickButton(testPad, PS4Controller.Button.kPS.value);

//   private final JoystickButton testShare = new JoystickButton(testPad, PS4Controller.Button.kShare.value);
//   private final JoystickButton testOptions = new JoystickButton(testPad, PS4Controller.Button.kOptions.value);

  public static final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  public static final SwerveRequest.FieldCentricFacingAngle facingAngle = new FieldCentricFacingAngle();

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  // public Command runAuto = drivetrain.getAutoPath("Test");
  // public Command goToCloseRedTrap = drivetrain.getAutoPath("Red Trap Close");
  // public Command goToFarRedTrap = drivetrain.getAutoPath("Red Trap Far");
  // public Command goToRedTrapCenter = drivetrain.getAutoPath("Red Trap Center");



  private void configureBindings() {
      SmartDashboard.putData("Pathfind to Stage Shot", AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("Shoot Underneath Stage"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  ));
   SmartDashboard.putData("pathfinding mid side", AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfinding mid side"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  ));

  SmartDashboard.putData("pathfind source side", AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfind source side"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  ));
    
   Command pathfindToAmp = AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("Amp Pathfinding"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  );

    SmartDashboard.putData("pathfind amp side stage", AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfind amp side stage"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  ));

  SmartDashboard.putData("Amp Pathfinding", AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("Amp Pathfinding"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0
  ));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(driverPad.interpolatedLeftYAxis() * Constants.Drive.MaxSpeed) // Drive
                                                                                            // forward
                                                                                                                         // with
            // negative Y (forward)
            .withVelocityY(driverPad.interpolatedLeftXAxis() * Constants.Drive.MaxSpeed) // Drive left with negative X
                                                                                          // (left)
            .withRotationalRate(
                // -driverPad.interpolatedRightXAxis() * Constants.Drive.MaxAngularRate
                Vision.getRotationToSpeaker()) // Drive counterclockwise with negative X (left)
            )
    );

    // Pilot Bindings
    driverOptions.whileTrue(drivetrain.run(() -> 
    RobotContainer.drivetrain.seedFieldRelative(new Pose2d(13.03, 4.12, new Rotation2d()))
    ));
    while (RobotContainer.driverPad.getShareButton()) { 
      AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("Amp Pathfinding"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0);
    }
 while (RobotContainer.driverPad.getTriangleButton()) { 
      AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfinding mid side"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0);
    }

     while (RobotContainer.driverPad.getCircleButton()) { 
      AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfind source side"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0);
    }
 while (RobotContainer.driverPad.getSquareButton()) { 
      AutoBuilder.pathfindThenFollowPath(
    PathPlannerPath.fromPathFile("pathfind amp side stage"), 
    new PathConstraints(
      4.0, 4.0, 
      Units.degreesToRadians(360), Units.degreesToRadians(540)
    ), 
    1.0);
    }



    // drivetrain.seedFieldRelative()
     // // reset the field-centric
                                                                                    // heading on options press
  

    // Co-Pilot Bindings

    operatorR1.whileTrue(new SpinUpSubwoofer());
    operatorPadButton.onTrue(new ActuateShooterToCLIMB());
    operatorR2.whileTrue(new ActuateRelease());
    operatorL1.whileTrue(new IndexerToShoot());
    operatorL2.whileTrue(new ActuateIntake());
    operatorShare.whileTrue(new ShootAMP());
    operatorStart.toggleOnTrue(new SetClimbManualOverride());
    operatorTriangle.whileTrue(new FeedFromMiddle());


    operatorStart.toggleOnTrue(new SetWristManualOverride());
    operatorCircle.onTrue(new ActuateToAndShootNote());
    
    


    // Pad For Development and Testing With One Controller
    // testStart.onTrue(new Actuate
    

  }

  public RobotContainer() {

    NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
    NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
    NamedCommands.registerCommand("print hello", Commands.print("hello"));
    NamedCommands.registerCommand("Start Intake", new StartIntakeAuto());
    NamedCommands.registerCommand("Actuate Wrist Far Side Speaker Shoot", new ActuateWristToSetpoint(47, 2));
    NamedCommands.registerCommand("Actuate Wrist Center Speaker", new ActuateWristToSetpoint(32.25, .5));
    NamedCommands.registerCommand("Actuate Wrist Far Side Stage", new ActuateWristToSetpoint(32, .5));
    NamedCommands.registerCommand("Align Wrist", new ActuateWristToAlign());
    NamedCommands.registerCommand("Spin Up Close", new SpinUpSubwooferAuto());
    NamedCommands.registerCommand("Indexer Starft Shoot", new IndexerStartShootAuto());
    NamedCommands.registerCommand("Indexer Stop Shoot", new IndexerStopShootAuto());
    NamedCommands.registerCommand("Stop Intake", new StopIntakeAuto());
    NamedCommands.registerCommand("Actuate Wrist Close", new ActuateWristToCloseShotAuto());
    NamedCommands.registerCommand("Actuate To Shoot Calculated", new ActuateToShootCalculatedAuto(.5));
    NamedCommands.registerCommand("Actuate To Shoot Fast", new ActuateToShootFastAuto());
    NamedCommands.registerCommand("Rotate To Speaker", new RotateToSpeakerAuto());
    NamedCommands.registerCommand("Actuate To And Shoot Note", new ActuateToAndShootNoteAuto());
        NamedCommands.registerCommand("0", new ActuateToAndFeedNote());


    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
