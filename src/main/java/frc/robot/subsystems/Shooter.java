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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	Timer timer = new Timer();

	public static double targetVelocity = 10;
	public static double setpointAmp = 0;
	boolean isAtSetpoint;

	public enum ShooterState {
		IDLE,
		CLOSESHOT,
		MANUAL,
		AMP,
		SOURCE,
		FEEDING,
		CLIMBING
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
		amper.setSmartCurrentLimit(10);
		ampEncoder.setPosition(0);


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
    output = ampController.calculate(ticksToDegrees(ampEncoder.getPosition()), setpointAmp)
	+ Constants.Shooter.AMP_KG * Math.cos(Math.toRadians(ticksToDegrees(ampEncoder.getPosition())))
	;
    amper.set(MathUtil.clamp(output, -.1, .1));

  }

	public void setWheelSpeed(double speed) {
		shooterLeft.set(speed);
	}

	public void setShooterVelocity(double RPS) {
		shooterLeft.setControl(voltageLeft.withVelocity(RPS).withFeedForward(Constants.Shooter.SHOOTER_FF));
	}

	public boolean isAtVelocitySetpoint(){
	return targetVelocity < shooterLeft.getRotorVelocity().getValueAsDouble();
	}

	@Override
	public void periodic() {
		timer.start();
		// System.out.println(isAtVelocitySetpoint());
		SmartDashboard.putNumber("RPS Shooter", shooterLeft.getRotorVelocity().getValueAsDouble());
		SmartDashboard.putBoolean("Shooter At Setpoint", shooterLeft.getRotorVelocity().getValueAsDouble() >= targetVelocity);

		SmartDashboard.putNumber("Current Output Left Shooter", shooterRight.getTorqueCurrent().getValueAsDouble());

		SmartDashboard.putString("Shooter State", String.valueOf(getState()));

		// SmartDashboard.putNumber("Amper Position", ampEncoder.getPosition());
		SmartDashboard.putNumber("Amper Current", amper.getOutputCurrent());

		// SmartDashboard.putNumber("Amper Setpoint", getSetpointAmp());

		// if (RobotState.isTeleop() && RobotState.isEnabled() && Vision.getDistanceToSpeaker() < 8){
		// 	shooterState = ShooterState.PASSIVE;
		// }

		switch (shooterState) {
			case IDLE:
				// if(Vision.getDistanceToSpeaker() < 8){
				// 	setShooterVelocity(10);
				// }
				// 	else{
						setWheelSpeed(0);
					// }
				setSetpointAmp(0);
				// if
				  setAmpPosition();
				break;
			case CLOSESHOT:
				  if(DriverStation.getAlliance().isPresent() && Vision.getDistanceToSpeaker() < 2.2){
					targetVelocity = 65;
					setShooterVelocity(150);
						}
				  else{
				targetVelocity = 70;
				setShooterVelocity(150);	
			}
				setSetpointAmp(0);
				 setAmpPosition();
				break;
			case AMP:
				setSetpointAmp(220);
				setAmpPosition();
				targetVelocity = 15;
				setShooterVelocity(35);
				break;
			case CLIMBING:
				setSetpointAmp(150);
				setAmpPosition();
				setWheelSpeed(0);
			case SOURCE:
				setShooterVelocity(-100);
				break;
			case MANUAL:
				// amper.set(RobotContainer.testPad.getRightX() * -.2);
				break;
			case FEEDING:
			setShooterVelocity(35);
				break;
		}
	}
}
