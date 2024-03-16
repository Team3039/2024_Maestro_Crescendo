// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.ActuateIntake;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public enum IntakeState{
       IDLE, 
       RELEASE,
      INTAKING,
      MANUAL
    }
    public IntakeState intakeState = IntakeState.MANUAL;

    public CANSparkMax intake = new CANSparkMax(Constants.Ports.INTAKE, CANSparkLowLevel.MotorType.kBrushless);

    public static double speed = 0;

    public static boolean hasIntaked = false;

  public Intake() {
    intake.setIdleMode(IdleMode.kBrake);
    intake.setInverted(false);
    intake.burnFlash();
    intake.setSmartCurrentLimit(500);
  }

  public void setState(IntakeState state) {
		intakeState = state;
	}

  public IntakeState getState() {
		return intakeState;
	}

	public void setWheelSpeed(double speed) {
		intake.set(speed);
	}

  @Override
  public void periodic() {
    SmartDashboard.putString("Intake State", String.valueOf(getState()));
    // SmartDashboard.putNumber("Intake Speed", intake.getOutputCurrent());
    // This method will be called once per scheduler run
    switch (intakeState) {
      case IDLE:
        setWheelSpeed(0);
        break;
      case RELEASE:
        setWheelSpeed(-.5);
        break;
      case INTAKING:
        if (Indexer.hasIndexed){
          setWheelSpeed(0);
        }
        else{
       setWheelSpeed(.65);
        }
        break;
      case MANUAL:
      if(RobotContainer.operatorPad.getTouchpad()){
      setWheelSpeed(0.5);
      }
    }
  }
}
