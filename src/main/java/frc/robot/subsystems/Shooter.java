// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake.IntakeState;

// public class Shooter extends SubsystemBase {

// 	public enum ShooterState {
		
// 		PASSIVE,
// 		CLOSESHOT,
//         FARSHOT
//   }
	
// 	public TalonFX shooterLeft = new TalonFX(Constants.Shooter.ShooterLeft);
// 	public TalonFX shooterRight = new TalonFX(Constants.Shooter.ShooterRight);

//     public ShooterState shooterState;



// 	public Shooter() {
		
// 		shooterLeft.setNeutralMode(NeutralModeValue.Coast);
// 		shooterRight.setNeutralMode(NeutralModeValue.Coast);
		

		
// 		shooterLeft.setInverted(true);
// 		shooterRight.setInverted(false);
		

		

// 		shooterLeft.getPosition().setUpdateFrequency(0);
// 		shooterRight.getPosition().setUpdateFrequency(0);
// 	}

	
// 	public void setState (ShooterState state){
// 		shooterState = state;
// 	}

// 	public ShooterState getState() {
// 		return shooterState;
// 	}

// 	public void setWheelSpeed(double speedLeft, double speedRight) {
// 		shooterLeft.set(speedLeft);
// 		shooterRight.set(speedRight);
// 	}



// 	@Override
// 	public void periodic() {

// 		SmartDashboard.putNumber("RPM Left Shooter", shooterLeft.getRotorVelocity().getValueAsDouble() * 60);
//         SmartDashboard.putNumber("RPM Right Shooter", shooterRight.getRotorVelocity().getValueAsDouble() * 60);

// 		SmartDashboard.putString("Shooter State", String.valueOf(getState()));




// 		switch (shooterState) {
// 			case PASSIVE:
// 				setWheelSpeed(0.2,0.2);
// 				break;
// 			case FARSHOT:
// 				setWheelSpeed(-0.7,-0.7);
// 				break;
// 			case CLOSESHOT:
// 				setWheelSpeed(0.5, 0.5);
// 				break;
//             default:
//             setWheelSpeed(0, 0);
// 		}
// 	}
// }
