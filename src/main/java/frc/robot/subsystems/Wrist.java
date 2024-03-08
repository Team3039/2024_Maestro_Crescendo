// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.MathUtils;
import frc.robot.util.Vector2;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  public enum WristState {
    MANUAL,
    ALIGN,
    POSITION,
    AMP,
    IDLE
  }

 public static WristState wristState = WristState.MANUAL;

  CANSparkMax wrist = new CANSparkMax(Constants.Ports.WRIST, MotorType.kBrushless);

  RelativeEncoder wristEncoder = wrist.getEncoder();

  SparkPIDController wristController = wrist.getPIDController();

  public static double setpointWrist = 0;

  boolean isAtSetpoint;

  public Wrist() {
    wrist.setInverted(false);
    wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.Wrist.Forward_Limit);
    wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.Wrist.Reverse_Limit);
    wristEncoder.setPosition(51.6);

 
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
    return Math.abs((setpointWrist - (wristEncoder.getPosition()))) <= tolerance;
  }

  public void getWristPosition() {
    wristEncoder.getPosition();
  }

  public void setWristPosition() {
    // wristController.setFF(Constants.Wrist.WRIST_KS);
    wristController.setReference(setpointWrist, ControlType.kPosition);
  }

  public double ticksToDegrees(double ticks) {
    double wristRotations = ticks * Constants.Wrist.WRIST_GEAR_RATIO;
    double wristDegrees = wristRotations * 360;
    return wristDegrees;
  } 


  @Override
  public void periodic() {
    // System.out.println(RobotContainer.indexer.getNoteDetected());

    SmartDashboard.putNumber("Wrist Speed", wrist.get());
    SmartDashboard.putString("WristState", String.valueOf(getState()));
    SmartDashboard.putNumber("Wrist Position Encoder", wristEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Position Degrees", ticksToDegrees(wristEncoder.getPosition()));
    SmartDashboard.putNumber("Setpoint Wrist", getSetpoint());

    switch (wristState) {
      case MANUAL:
        wrist.set(RobotContainer.operatorPad.getRightY() * -1); // intuitive
        break;
      case POSITION:
        setWristPosition();
        break;
      case ALIGN:
        setSetpointWrist(30);
        setWristPosition();
        break;
      case AMP:
        setSetpointWrist(-20);
        setWristPosition();
        break;
      case IDLE:
        setSetpointWrist(15);
        setWristPosition();
        break;
      default:
        wrist.set(0);
        break;
    }
  }
}
