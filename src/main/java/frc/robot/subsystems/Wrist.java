// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  static WristState wristState = WristState.MANUAL;

  CANSparkMax wrist = new CANSparkMax(Constants.Ports.WRIST, MotorType.kBrushless);

  AbsoluteEncoder wristEncoder = wrist.getAbsoluteEncoder();

  SparkPIDController wristController = wrist.getPIDController();

  
  double setpoint = 0;

  boolean isAtSetpoint;

  double setpointWrist = 0;
  
    
  public Wrist() {
    wrist.setInverted(false);
    wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.Wrist.Forward_Limit);
    wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.Wrist.Reverse_Limit);
    

    wristEncoder.setZeroOffset(setpoint);
    wristController.setP(Constants.Wrist.WRIST_KP);
    wristController.setI(Constants.Wrist.WRIST_KI);
    wristController.setD(Constants.Wrist.WRIST_KD);
  




    wrist.burnFlash();
  }

  public enum WristState{
    MANUAL, 
    IDLE, 
    POSITION,
    AMP,
    INDEXING
  }

   public static WristState getState() {
    return wristState;
  }

  public void setState(WristState state) {
    wristState = state;
  }

  public double getSetpoint(){
    return setpointWrist;
  }

  public void setSetpointWrist(double setpoint){
    setpointWrist = setpoint;
  }

  public void getWristPosition(){
  }
  
  


  public void setWristPosition(double setpoint){
    wristController.setFF(Constants.Wrist.K_FF);
    wristController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(wristState){
     case MANUAL:
      break;
     case POSITION:
      break;
    case INDEXING:
    break;
    case AMP:
      break;
    case IDLE:
      break;
    }
  }
}
