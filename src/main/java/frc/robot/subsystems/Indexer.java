// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import edu.wpi.first.wpilibj.DigitalInput;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Indexer extends SubsystemBase {
//   /** Creates a new Flywheel. */
//   public enum IndexerState{
//        IDLE, 
//        SHOOTING,
//       INTAKING
//     }
//     public IndexerState indexerState = IndexerState.IDLE;

//     public CANSparkMax indexer = new CANSparkMax(Constants.Indexer.Indexer, MotorType.kBrushless);

//     public DigitalInput beamBreak1;


//     public static double speed = 0;

//   public Indexer() {
//     indexer.setIdleMode(IdleMode.kBrake);

//     indexer.setInverted(false);
//     beamBreak1 = new DigitalInput(Constants.Indexer.BeamBreak1);
	
//   }

//   public void setState(IndexerState state) {
// 		indexerState = state;
// 	}

//   public IndexerState getState() {
// 		return indexerState;
// 	}

// 	public void setWheelSpeed(double speed) {
// 		indexer.set(speed);
// 	}

//     public boolean getNoteDetected(){
//         return beamBreak1.get();
//     }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     switch (indexerState) {
//       case IDLE:
//         setWheelSpeed(0);
//       break;
//       case SHOOTING:
//         setWheelSpeed(.6);
//         break;
//       case INTAKING:
//        setWheelSpeed(.6);
//         break;
//     }
//   }
// }
