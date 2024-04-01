// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {

	public enum ClimbState {
		IDLE,
		MANUAL,
		POSITION,
		CLIMB_UP,
	}

	public ClimbState climbState = ClimbState.IDLE;

	public CANSparkMax climbA = new CANSparkMax(Constants.Ports.CLIMB, MotorType.kBrushless);
	// public CANSparkMax climbB = new CANSparkMax(Constants.Ports.CLIMB_B,
	// MotorType.kBrushless);

	public RelativeEncoder encoder = climbA.getEncoder();

	private PIDController controller = new PIDController(
			Constants.Climb.CLIMB_KP,
			Constants.Climb.CLIMB_KI,
			Constants.Climb.CLIMB_KD);

	// neo rotations
	public static double setpointClimb = 0;

	public Climb() {

		climbA.setIdleMode(IdleMode.kBrake);

		climbA.setInverted(true);
		// climbB.setInverted(true);

		climbA.enableSoftLimit(SoftLimitDirection.kForward, false);
		climbA.enableSoftLimit(SoftLimitDirection.kReverse, true);
		climbA.setSoftLimit(SoftLimitDirection.kForward, 22);
		climbA.setSoftLimit(SoftLimitDirection.kReverse, 0);
		// climbB.follow(climbA);

		climbA.burnFlash();
		// climbB.burnFlash();

		controller.setTolerance(3);
	}

	public void setState(ClimbState state) {
		climbState = state;
	}

	public ClimbState getState() {
		return climbState;
	}

	public void setClimbOpenLoop(double percent) {
		climbA.set(percent);
	}

	public void setClimbClosedLoop() {
		double output = 0;

		output = controller.calculate(encoder.getPosition(), setpointClimb) + Constants.Climb.CLIMB_KS;
		climbA.set(MathUtil.clamp(output, -.2, .7));
	}

	public static double getSetpoint() {
		return setpointClimb;
	}

	public static void setSetpoint(double setpoint) {
		setpointClimb = setpoint;
	}

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointClimb - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climb Current Draw", climbA.getOutputCurrent());
		SmartDashboard.putNumber("Climb Encoder", encoder.getPosition());
		SmartDashboard.putString("Climb State", String.valueOf(getState()));
		SmartDashboard.putNumber("Climb Output", climbA.get());
		SmartDashboard.putNumber("Setpoint Climb", getSetpoint());
		switch (climbState) {
			case IDLE:
				setSetpoint(0);
				setClimbClosedLoop();
				break;
			case MANUAL:
				setClimbOpenLoop(-1 * RobotContainer.operatorPad.getLeftY());// intuitive
				break;
			case POSITION:
				setClimbClosedLoop();
				break;
			case CLIMB_UP:
				setSetpoint(16);
				setClimbClosedLoop();
				break;
		}
	}
}
