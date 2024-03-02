// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;

public class Shooter extends SubsystemBase {

	public enum ShooterState {
		
		PASSIVE,
		CLOSESHOT,
        FARSHOT
  }
	
	public TalonFX shooterLeft = new TalonFX(Constants.Ports.SHOOTER_LEFT);
	public TalonFX shooterRight = new TalonFX(Constants.Ports.SHOOTER_RIGHT);

    public ShooterState shooterState = ShooterState.PASSIVE;

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

	
	public void setState (ShooterState state){
		shooterState = state;
	}

	public ShooterState getState() {
		return shooterState;
	}

	public void setWheelSpeed(double speedLeft, double speedRight) {
		shooterLeft.set(speedLeft);
		shooterRight.set(speedRight);
	}
	public void setShooterVelocity(double RPMLeft, double RPMRight){
        shooterLeft.setControl(voltageLeft.withVelocity(RPMLeft));
		shooterRight.setControl(voltageRight.withVelocity(RPMRight));
    }



	@Override
	public void periodic() {

		setShooterVelocity(500, 500);

		SmartDashboard.putNumber("RPM Left Shooter", shooterLeft.getRotorVelocity().getValueAsDouble() );
        SmartDashboard.putNumber("RPM Right Shooter", shooterRight.getRotorVelocity().getValueAsDouble());
	
		SmartDashboard.putString("Shooter State", String.valueOf(getState()));




		switch (shooterState) {
			case PASSIVE:
			setShooterVelocity(40, 40);
				break;
			case FARSHOT:
				setWheelSpeed(-0.7,-0.7);
				break;
			case CLOSESHOT:
				setWheelSpeed(0.5, 0.5);
				break;
            default:
            setWheelSpeed(20, 20);
		}
	}
}
