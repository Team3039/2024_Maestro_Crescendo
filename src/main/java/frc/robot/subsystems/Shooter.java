// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
Timer timer = new Timer();
public static double targetVelocity;
	public enum ShooterState {
		IDLE,
		PASSIVE,
		CLOSESHOT,
		INTERPOLATED,
		AMP,
		SOURCE
	}

	public TalonFX shooterLeft = new TalonFX(Constants.Ports.SHOOTER_LEFT);
	public TalonFX shooterRight = new TalonFX(Constants.Ports.SHOOTER_RIGHT);

	public Servo amper = new Servo(Constants.Ports.AMPER);
	

	public ShooterState shooterState = ShooterState.IDLE;

	VelocityVoltage voltageLeft = new VelocityVoltage(0);
	VelocityVoltage voltageRight = new VelocityVoltage(0);

	public Shooter() {
	CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(70);

		shooterLeft.setNeutralMode(NeutralModeValue.Coast);
		shooterRight.setNeutralMode(NeutralModeValue.Coast);

		Slot0Configs configs = new Slot0Configs()
				.withKP(Constants.Shooter.SHOOTER_KP)
				.withKI(Constants.Shooter.SHOOTER_KI)
				.withKD(Constants.Shooter.SHOOTER_KD);

		shooterLeft.getConfigurator().apply(configs);
		shooterLeft.getConfigurator().apply(currentLimit);
		shooterRight.getConfigurator().apply(configs);
		shooterRight.getConfigurator().apply(currentLimit);

		shooterLeft.setInverted(true);
		shooterRight.setInverted(false);

		shooterLeft.getPosition().setUpdateFrequency(0);
		shooterRight.getPosition().setUpdateFrequency(0);
		shooterRight.setControl(new StrictFollower(shooterLeft.getDeviceID()));
	}

	public void setState(ShooterState state) {
		shooterState = state;
	}

	public ShooterState getState() {
		return shooterState;
	}

	public void setWheelSpeed(double speed) {
		shooterLeft.set(speed);
	}

	public void setShooterVelocity(double RPS) {
		shooterLeft.setControl(
			voltageLeft.withVelocity(RPS).
			withFeedForward(Constants.Shooter.SHOOTER_FF));
	}
	// public boolean isAtSetpoint(double RPS){
	// if ()
	// return false;
	// }

	@Override
	public void periodic() {
		timer.start();

		SmartDashboard.putNumber("RPS Shooter", shooterLeft.getRotorVelocity().getValueAsDouble());
		SmartDashboard.putBoolean("Shooter At Setpoint", shooterLeft.getRotorVelocity().getValueAsDouble() >= targetVelocity);

		SmartDashboard.putNumber("Current Output Left Shooter", shooterRight.getTorqueCurrent().getValueAsDouble());

		SmartDashboard.putString("Shooter State", String.valueOf(getState()));

		switch (shooterState) {
			case IDLE:
				targetVelocity = 5;
				timer.reset();
				// setShooterVelocity(0);
				setWheelSpeed(0);
				if(timer.get() > 1.50){
					amper.set(0.0);
				}
				break;
			case PASSIVE:
				targetVelocity = 10;
				setShooterVelocity(20);
				break;
			case INTERPOLATED:
				// setShooterVelocity(RobotContainer.vision.setpointShooter);
				break;
			case CLOSESHOT:
				targetVelocity = 60;
			setShooterVelocity(100);
			break;
			case AMP:
				targetVelocity = 15;
				setShooterVelocity(28);
				amper.setPosition(1.0);
				break;
			case SOURCE:
				targetVelocity = -4;
				setShooterVelocity(-3);
				break;
		}
	}
}
