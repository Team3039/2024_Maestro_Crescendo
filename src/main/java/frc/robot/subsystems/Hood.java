// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkBase.SoftLimitDirection;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;

// public class Hood extends SubsystemBase {

//   public enum HoodState {
//     IDLE,
//     MANUAL,
//     POSITION,
        // TRACKING
//   }

//   public HoodState hoodState = HoodState.IDLE;

//   CANSparkMax hood = new CANSparkMax(1, MotorType.kBrushless);
//   public ArmFeedforward feedForward = new ArmFeedforward(
//       Constants.Hood.HOOD_KS,
//       Constants.Hood.HOOD_KG,
//       Constants.Hood.HOOD_KV);

//   public ProfiledPIDController profiledController = new ProfiledPIDController(
//       Constants.Hood.HOOD_KP,
//       Constants.Hood.HOOD_KI,
//       Constants.Hood.HOOD_KD,
//       new TrapezoidProfile.Constraints(
//           Constants.Hood.HOOD_MAX_VEL,
//           Constants.Hood.HOOD_MAX_ACCEL));

//   public PIDController controller = new PIDController(
//       Constants.Hood.HOOD_KP,
//       Constants.Hood.HOOD_KI,
//       Constants.Hood.HOOD_KD);

//   public static double setpointHood = 0;
//   public static double idleSetpoint = 0;

//   double hoodSetpointOffset = 0;

//   public Hood() {
//     // hood.configFactoryDefault();%
//     hood.setIdleMode(IdleMode.kBrake);

  
//      hood.setInverted(false);
//     // hood.setSensorPhase(true);
    
//     hood.enableSoftLimit(SoftLimitDirection.kForward, true);
// 		hood.enableSoftLimit(SoftLimitDirection.kReverse, true);
// 		hood.setSoftLimit(SoftLimitDirection.kForward, 75);
// 		hood.setSoftLimit(SoftLimitDirection.kReverse, 0);

//     hood.burnFlash();
//     controller.setTolerance(3);
// 		profiledController.setTolerance(3);
//   }

//   public HoodState getState() {
//     return hoodState;
//   }

//   public void setState(HoodState state) {
//     hoodState = state;
//   }

//   public double degreesToTicks(double degrees) {
//     double armRotations = degrees / 360;
//     double ticks = armRotations * 84;
//     return ticks;
//   }

//   // give the encoder value to get degrees
//   public double ticksToDegrees(double ticks) {
//     double armRotations = ticks / 84;
//     double armDegrees = armRotations * 360;
//     return armDegrees;
//   }

 
//   public void setHoodPosition(boolean isProfiled) {
//     if (isProfiled) {
//       profiledController.setGoal(setpointHood);
//       // hood.set(ControlType.kPosition, degreesToTicks(degrees));
//       hood.set( profiledController.calculate(hood.getEncoder().getPosition()) +
//           feedForward.calculate(Math.toRadians(profiledController.getSetpoint().position),
//               profiledController.getSetpoint().velocity));
//     } else {
//       hood.set(MathUtil.clamp(controller.calculate(
//           ticksToDegrees(hood.getEncoder().getPosition()),
//           setpointHood), -.25, .3));
//     }
//   }

//   public void setHoodPercent(double percent) {
//     hood.set(percent +
//         Math.cos(Math.toRadians(ticksToDegrees(hood.getEncoder().getPosition()))) * Constants.Hood.HOOD_KG +
//         Constants.Hood.HOOD_KS);
//   }

//   public static double getSetpoint() {
//     return setpointHood;
//   }

//   public static void setSetpoint(double setpoint) {
//     setpointHood = setpoint;
//   }

//   public double getHoodPosition() {
//     return ticksToDegrees(hood.getEncoder().getPosition());
//   }

// 	public boolean isAtSetpoint(boolean isProfiled, double tolerance) {
// 		return Math.abs((setpointHood - ticksToDegrees(hood.getEncoder().getPosition()))) <= tolerance;
// 	}

//   public double getHoodOffset() {
//     return hoodSetpointOffset;
//   }

//   public void changeHoodOffset(double offset) {
//     hoodSetpointOffset += offset;
//   }

//   @Override
//   public void periodic() {
//     // SmartDashboard.putNumber("Hood Absolute Encoder",
//     // hood.getSelectedSensorPosition());
//     // System.out.println(ticksToDegrees(hood.getSelectedSensorPosition()));
//     // System.out.println(hood.getMotorOutputPercent());
//     // SmartDashboard.putNumber("Hood Currnent Input", hood.getSupplyCurrent());
//     // SmartDashboard.putNumber("Hood Current Output", hood.getStatorCurrent());
//     // System.out.println(setpointHood);
//     SmartDashboard.putNumber("Hood Angle", getHoodPosition());
//     // SmartDashboard.putString("Hood State", String.valueOf(getState()));
//     // SmartDashboard.putNumber("Hood Offset", getHoodOffset());
//     // SmartDashboard.putNumber("Hood Setpoint", getSetpoint());
//     idleSetpoint = getState().equals(HoodState.IDLE) ? idleSetpoint : setpointHood;
    

//     switch (hoodState) {
//       case IDLE:
//         if (getState().equals(HoodState.IDLE)) {
//         setSetpoint(-20);
//         setHoodPosition(false);
//         }
//         else {
//         setSetpoint(idleSetpoint);
//         setHoodPosition(false);
//         }
//         break;
//       case MANUAL:
//         setHoodPercent(RobotContainer.driverPad.getRightY() * .2);
//         break;
//       case POSITION:
//         setHoodPosition(false);
//         break;
//     }
//   }
// }
