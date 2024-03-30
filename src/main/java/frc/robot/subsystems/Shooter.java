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
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	Timer timer = new Timer();

	public static double targetVelocity;
	public static double setpointAmp = 0;
	boolean isAtSetpoint;

	public enum ShooterState {
		IDLE,
		PASSIVE,
		CLOSESHOT,
		MANUAL,
		AMP,
		SOURCE
	}

	public TalonFX shooterLeft = new TalonFX(Constants.Ports.SHOOTER_LEFT);
	public TalonFX shooterRight = new TalonFX(Constants.Ports.SHOOTER_RIGHT);

	public CANSparkMax amper = new CANSparkMax(Constants.Ports.AMPER, MotorType.kBrushless);
	RelativeEncoder ampEncoder = amper.getEncoder();

	private PIDController ampController = new PIDController(
			Constants.Shooter.AMP_KP,
			Constants.Shooter.AMP_KI,
			Constants.Shooter.AMP_KD);

	public ShooterState shooterState = ShooterState.IDLE;

	VelocityVoltage voltageLeft = new VelocityVoltage(0);
	VelocityVoltage voltageRight = new VelocityVoltage(0);

	public Shooter() {
		CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(70);

		amper.setInverted(true);
		amper.enableSoftLimit(SoftLimitDirection.kForward, true);
		amper.enableSoftLimit(SoftLimitDirection.kReverse, true);

		amper.setSoftLimit(SoftLimitDirection.kForward, Constants.Shooter.AMP_FORWARD_LIMIT);
		amper.setSoftLimit(SoftLimitDirection.kReverse, Constants.Shooter.AMP_REVERSE_LIMIT);

		ampController.setP(Constants.Shooter.AMP_KP);
		ampController.setI(Constants.Shooter.AMP_KI);
		ampController.setD(Constants.Shooter.AMP_KD);

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

		amper.burnFlash();

	}

	public void setState(ShooterState state) {
		shooterState = state;
	}

	public ShooterState getState() {
		return shooterState;
	}
	public double ticksToDegrees(double ticks) {
		double ampRotations = ticks * Constants.Shooter.AMP_GEAR_RATIO;
		double ampDegrees = ampRotations * 360.0;
		return ampDegrees;
	  }

	public double getSetpointAmp() {
		return setpointAmp;
	}

	public void setSetpointAmp(double setpoint) {
		setpointAmp = setpoint;
	}
	
	 public void setAmpPosition() {
    double output = 0;
    output = ampController.calculate(ticksToDegrees(ampEncoder.getPosition()), setpointAmp);
    amper.set(MathUtil.clamp(output, -.4, .4));
  }

	public void setWheelSpeed(double speed) {
		shooterLeft.set(speed);
	}

	public void setShooterVelocity(double RPS) {
		shooterLeft.setControl(
				voltageLeft.withVelocity(RPS).withFeedForward(Constants.Shooter.SHOOTER_FF));
	}

	// public boolean isAtSetpoint(double RPS){
	// if ()
	// return false;
	// }

	@Override
	public void periodic() {
		timer.start();
		// System.out.println(ampEncoder.getPosition());
		SmartDashboard.putNumber("RPS Shooter", shooterLeft.getRotorVelocity().getValueAsDouble());
		SmartDashboard.putBoolean("Shooter At Setpoint",
				shooterLeft.getRotorVelocity().getValueAsDouble() >= targetVelocity);

		SmartDashboard.putNumber("Current Output Left Shooter", shooterRight.getTorqueCurrent().getValueAsDouble());

		SmartDashboard.putString("Shooter State", String.valueOf(getState()));

		SmartDashboard.putNumber("Amp Position", ampEncoder.getPosition());
		SmartDashboard.putNumber("Amp Setpoint", getSetpointAmp());

		switch (shooterState) {
			case IDLE:
				setSetpointAmp(3);
				setAmpPosition();
				targetVelocity = 5;
				timer.reset();
				setShooterVelocity(0);
				setWheelSpeed(0);
				if(timer.get() > 1.50 && ampEncoder.getPosition() > 3){
				amper.set(-0.2);
				}
				break;
			case PASSIVE:
				targetVelocity = 10;
				setShooterVelocity(20);
				break;
			case CLOSESHOT:
				targetVelocity = 60;
				setShooterVelocity(100);
				break;
			case AMP:
				targetVelocity = 15;
				setShooterVelocity(28);
				if (ampEncoder.getPosition() < 15) {
					amper.set(.2);
				}
				break;
			case SOURCE:
				targetVelocity = -4;
				setShooterVelocity(-3);
				break;
			case MANUAL:
				amper.set(RobotContainer.testPad.getRightX() * .5);
				break;
		}
	}
}
