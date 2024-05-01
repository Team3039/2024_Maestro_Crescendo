// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  double wristHeight = .15; //meters, tune this value

  public enum WristState {
    MANUAL,
    POSITION,
    AMP,
    ALIGN,
    CLOSESHOT,
    TUNABLE,
    ESTIMATED,
    FEEDING,
    SOURCING

  }

  public static WristState wristState = WristState.CLOSESHOT;

  CANSparkMax wrist = new CANSparkMax(Constants.Ports.WRIST, MotorType.kBrushless);

  RelativeEncoder wristEncoder = wrist.getEncoder();

  private PIDController wristController = new PIDController(
      Constants.Wrist.WRIST_KP,
      Constants.Wrist.WRIST_KI,
      Constants.Wrist.WRIST_KD);

  public static double setpointWrist = 0;

  boolean isAtSetpoint;

  public Wrist() {
    wrist.setInverted(false);
    wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
    wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);

    wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.Wrist.Forward_Limit);
    wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.Wrist.Reverse_Limit);
    wristEncoder.setPosition(55.6);

    wristController.setP(Constants.Wrist.WRIST_KP);
    wristController.setI(Constants.Wrist.WRIST_KI);
    wristController.setD(Constants.Wrist.WRIST_KD);

    wrist.burnFlash();
  }

  public static WristState getState() {
    return wristState;
  }

  public void setState(WristState state) {
    wristState = state;
  }

  public double getSetpoint() {
    return setpointWrist;
  }

  public void setSetpointWrist(double setpoint) {
    setpointWrist = setpoint;
  }

  public boolean isAtSetpoint(double tolerance) {
    return Math.abs((setpointWrist - (ticksToDegrees(wristEncoder.getPosition())))) <= tolerance;
  }

  public void getWristPosition() {
    wristEncoder.getPosition();
  }
  
  public double getCalculatedPosition() {
    if(DriverStation.getAlliance().isPresent() ){
     wristHeight = 0.55;
      
      return degreesToTicks(Math.toDegrees(Math.atan((Vision.SpeakerCenterBlue.getZ() - wristHeight) / Vision.getDistanceToSpeaker()))); 
    }
    return 0;
  }

  public void setWristPosition() {
    double output = 0;
    output = wristController.calculate(ticksToDegrees(wristEncoder.getPosition()), setpointWrist)
        + Constants.Wrist.WRIST_KS;
    wrist.set(MathUtil.clamp(output, -.4, .4));
  }

  public double ticksToDegrees(double ticks) {
    double wristRotations = ticks * Constants.Wrist.WRIST_GEAR_RATIO;
    double wristDegrees = wristRotations * 360.0;
    return wristDegrees;
  }
  public double degreesToTicks(double wristDegrees) {
   return wristDegrees / (Constants.Wrist.WRIST_GEAR_RATIO * 360.0);
  }
 
  @Override
  public void periodic() {
    // System.out.println(RobotContainer.indexer.getNoteDetected());

    // SmartDashboard.putNumber("Wrist Speed", wrist.get());
    SmartDashboard.putString("WristState", String.valueOf(getState()));
    // SmartDashboard.putNumber("Wrist Position Encoder", wristEncoder.getPosition());
    // SmartDashboard.putNumber("Wrist Position Degrees",
    // ticksToDegrees(wristEncoder.getPosition()));

    // SmartDashboard.putNumber("Setpoint Wrist", getSetpoint());
    SmartDashboard.putBoolean("Wrist At Setpoint", (Math.abs(getCalculatedPosition() - wristEncoder.getPosition()) < .3));

    // SmartDashboard.putNumber("Calculated Pos Wrist", getCalculatedPosition());


    switch (wristState) {
      case MANUAL:
        wrist.set(RobotContainer.operatorPad.getRightY() * -1); // intuitive
        break;
      case FEEDING:
      setSetpointWrist(34);
      setWristPosition();
      case POSITION:
        setWristPosition();
        break;
      case AMP:
        setSetpointWrist(55);
        setWristPosition();
        break;
      case ALIGN:
        setSetpointWrist(ticksToDegrees(Constants.Wrist.WRIST_INTAKING));
        setWristPosition();
        break;
      case CLOSESHOT:
        setSetpointWrist(ticksToDegrees(52));
        setWristPosition();
        break;
      case TUNABLE:
      // if(RobotContainer.testPad.getL1Button()){
      //  setpointWrist += .2;
      // }
      // if(RobotContainer.testPad.getR1Button()){
      //   setpointWrist -= .2;
      // }
      // setWristPosition();
        break;
      case ESTIMATED:  
      setSetpointWrist(getCalculatedPosition());
      setWristPosition();;
        break;
        case SOURCING:
        setSetpointWrist(26);
        setWristPosition();
    }
  }

 
}
