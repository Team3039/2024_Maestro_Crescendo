// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

	public enum ShooterState {
		IDLE,
		PASSIVE,
		CLOSESHOT,
		FARSHOT,
		POSITION,
		TEST

	}

	public TalonFX shooterLeft = new TalonFX(Constants.Ports.SHOOTER_LEFT);
	public TalonFX shooterRight = new TalonFX(Constants.Ports.SHOOTER_RIGHT);

	public ShooterState shooterState = ShooterState.IDLE;

	VelocityVoltage voltageLeft = new VelocityVoltage(0);
	VelocityVoltage voltageRight = new VelocityVoltage(0);

	public Shooter() {
		shooterLeft.setNeutralMode(NeutralModeValue.Coast);
		shooterRight.setNeutralMode(NeutralModeValue.Coast);

		Slot0Configs configs = new Slot0Configs()
				.withKP(Constants.Shooter.SHOOTER_KP)
				.withKI(Constants.Shooter.SHOOTER_KI)
				.withKD(Constants.Shooter.SHOOTER_KD);

		shooterLeft.getConfigurator().apply(configs);
		shooterRight.getConfigurator().apply(configs);

		shooterLeft.setInverted(true);
		shooterRight.setInverted(false);

		shooterLeft.getPosition().setUpdateFrequency(0);
		shooterRight.getPosition().setUpdateFrequency(0);
	}

	public void setState(ShooterState state) {
		shooterState = state;
	}

	public ShooterState getState() {
		return shooterState;
	}

	public void setWheelSpeed(double speed) {
		shooterLeft.set(speed);
		shooterRight.set(speed);
	}

	public void setShooterVelocity(double RPS) {
		shooterLeft.setControl(voltageLeft.withVelocity(RPS));
		shooterRight.setControl(voltageRight.withVelocity(RPS));
	}
	// public boolean isAtSetpoint(double RPS){
	// if ()
	// return false;
	// }

	@Override
	public void periodic() {

		SmartDashboard.putNumber("RPM Left Shooter", shooterLeft.getRotorVelocity().getValueAsDouble());
		SmartDashboard.putNumber("RPM Right Shooter", shooterRight.getRotorVelocity().getValueAsDouble());

		SmartDashboard.putString("Shooter State", String.valueOf(getState()));

		switch (shooterState) {
			case IDLE:
				setShooterVelocity(50);
				// setWheelSpeed(.5);
				break;
			case PASSIVE:
				setShooterVelocity(40);
				break;
			case POSITION:
				break;
			case FARSHOT:
				setWheelSpeed(-0.7);
				break;
			case CLOSESHOT:
				setWheelSpeed(0.6);
				break;
			case TEST:
				if (RobotContainer.operatorPad.getL3Button()) {
					setWheelSpeed(0.3);
				}
				break;
		}
	}
}
