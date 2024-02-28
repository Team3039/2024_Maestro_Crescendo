// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Climb extends SubsystemBase {
//   /** Creates a new Climb. */
//   double setpoint = 0;
//   public enum ClimbState{
//     IDLE,
//     MIDCHAIN,
//     MANUAL
//   }
//   ClimbState climbState = ClimbState.IDLE;

//   CANSparkMax leftClimber = new CANSparkMax(0, MotorType.kBrushless);
//   CANSparkMax rightClimber = new CANSparkMax(0, MotorType.kBrushless);

//     public RelativeEncoder encoderLeft = leftClimber.getEncoder();
//     public RelativeEncoder encoderRight = leftClimber.getEncoder();


//   public Climb() {


//     leftClimber.setIdleMode(IdleMode.kCoast);
//     rightClimber.setIdleMode(IdleMode.kCoast);

//     leftClimber.config_kP(0, 0.8);
//     leftClimber.config_kI(0, 0);
//     leftClimber.config_kD(0, 0);

//     rightClimber.config_kP(0, 0.8);
//     rightClimber.config_kI(0, 0);
//     rightClimber.config_kD(0, 0);

// 		rightClimber.follow(leftClimber, true);

//     leftClimber.setSoftLimit(null, 0);
//     leftClimber.setSoftLimit(null, 0);
//     rightClimber.setSoftLimit(null, 0);
//     rightClimber.setSoftLimit(null, 0);


//   }

//   public double getSetpoint(){
//     return setpoint;
//   }

//   public double setSetpoint(double setpoint){
   
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     switch(climbState){
//   case IDLE:
//   break;
//   case MIDCHAIN:
//   break;
//   case MANUAL:
//   break;
// }
//   }
// }
