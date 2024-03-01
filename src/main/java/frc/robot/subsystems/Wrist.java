// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.Vector2;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  static WristState wristState = WristState.MANUAL;
  CANSparkMax wrist = new CANSparkMax(1, MotorType.kBrushless);

  RelativeEncoder encoder = wrist.getEncoder();
  
  double setpoint = 0;

  PIDController controller = new PIDController
  (Constants.Wrist.K_P,
  Constants.Wrist.K_I, 
   Constants.Wrist.K_D);



  boolean isAtSetpoint;
  double setpointWrist = 0;
  
    
  public Wrist() {
    wrist.setInverted(false);
    wrist.setSoftLimit(SoftLimitDirection.kForward, Constants.Wrist.Forward_Limit);
    wrist.setSoftLimit(SoftLimitDirection.kReverse, Constants.Wrist.Reverse_Limit);

    PIDController wristController = new PIDController(
      Constants.Wrist.K_P, 
      Constants.Wrist.K_I, 
       Constants.Wrist.K_D);



    wrist.burnFlash();
  }

  public enum WristState{
    MANUAL, 
    IDLE, 
    POSITION,
    AMP
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

  public double getWristPosition(){
   return encoder.getPosition();
  }

  public void setWristPosition(double setpoint){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(wristState){
     case MANUAL:
      break;
     case POSITION:
      break;
    case AMP:
      break;
    case IDLE:
      break;
    }
  }
}
