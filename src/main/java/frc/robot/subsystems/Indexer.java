// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Indexer extends SubsystemBase {
  public enum IndexerState{
       IDLE, 
       SHOOTING,
      INDEXING,
      MANUAL
    }
    public IndexerState indexerState = IndexerState.IDLE;

    public CANSparkMax indexer = new CANSparkMax(Constants.Ports.INDEXER, MotorType.kBrushless);

    public DigitalInput beamBreak = new DigitalInput(Constants.Ports.BEAM_BREAK);

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
    switch (indexerState) {
      case IDLE:
        setWheelSpeed(0.0);
      break;
      case SHOOTING:
        setWheelSpeed(.9);
        break;
      case INDEXING:
       setWheelSpeed(.6);
        break;
      case MANUAL:
        if (RobotContainer.operatorPad.getPSButton()){
          setWheelSpeed(0.6);
        }
    }
  }
}
