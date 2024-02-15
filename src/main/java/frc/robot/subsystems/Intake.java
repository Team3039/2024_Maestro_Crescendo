// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public enum IntakeState{
       IDLE, 
       RELEASE,
      INTAKING
    }
    public IntakeState intakeState = IntakeState.IDLE;

    public CANSparkMax spinner = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless);

    public static double speed = 0;

  public Intake() {
    spinner.setIdleMode(IdleMode.kBrake);

    spinner.setInverted(true);
    spinner.burnFlash();
	
  }

  public void setState(IntakeState state) {
		intakeState = state;
	}

  public IntakeState getState() {
		return intakeState;
	}

	public void setWheelSpeed(double speed) {
		spinner.set(speed);
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (intakeState) {
      case IDLE:
        setWheelSpeed(0);
      break;
      case RELEASE:
        setWheelSpeed(-.8);
        break;
      case INTAKING:
       setWheelSpeed(.8);
        break;
    }
  }
}
