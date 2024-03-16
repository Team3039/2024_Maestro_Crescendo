// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Indexer extends SubsystemBase {
  public enum IndexerState{
       IDLE, 
       SHOOTING,
      INDEXING,
      MANUAL,
      RELEASE,
      SOURCING
    }
    public IndexerState indexerState = IndexerState.IDLE;

    public CANSparkMax indexer = new CANSparkMax(Constants.Ports.INDEXER, MotorType.kBrushless);

    public DigitalInput beamBreak = new DigitalInput(Constants.Ports.BEAM_BREAK);

   public static boolean hasIndexed = false;

    public boolean hasNote;

    public static double speed = 0;

  public Indexer() {
    indexer.setIdleMode(IdleMode.kBrake);

    indexer.setInverted(false);
	
  }

  public void setState(IndexerState state) {
		indexerState = state;
	}

  public IndexerState getState() {
		return indexerState;
	}

	public void setWheelSpeed(double speed) {
		indexer.set(speed);
	}

  public boolean getNoteDetected(){
      return beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putBoolean("hasNote", getNoteDetected());
      SmartDashboard.putString("Indexer State", String.valueOf(getState()));
      SmartDashboard.putBoolean("hasIndexed", hasIndexed);
    switch (indexerState) {
      case IDLE:
        setWheelSpeed(0.0);
        hasIndexed = false;
      break;
      case SHOOTING:
      hasIndexed = false;
      // if (RobotContainer.shooter.shooterState == ShooterState.AMP){
      //   setWheelSpeed(.3);
      // }
      //  else{ 
        setWheelSpeed(.8);
      // }
        break;
      case INDEXING:
      if (beamBreak.get()){
        hasIndexed = true;
      }
      if (hasIndexed){
        setWheelSpeed(0);
      }
      else{
      setWheelSpeed(.6);
      }
        break;
      case MANUAL:
        if (RobotContainer.operatorPad.getPSButton()){
          setWheelSpeed(0.6);
        }
      break;
      case RELEASE:
        hasIndexed = false;
      setWheelSpeed(-.5);
      break;
      case SOURCING:
        setWheelSpeed(-.4);
        hasIndexed = false;
        break;
    }
  }
}
