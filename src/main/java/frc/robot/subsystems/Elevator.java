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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION,
		AMPING,
		CLIMBING
	}

	public ElevatorState elevatorState = ElevatorState.IDLE;

	public CANSparkMax elevatorA = new CANSparkMax(Constants.Ports.ELEVATOR_A, MotorType.kBrushless);
	// public CANSparkMax elevatorB = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);

	public RelativeEncoder encoder = elevatorA.getEncoder();

	private PIDController controller = new PIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD);

	// neo rotations
	public static double setpointElevator = 0;

	public Elevator() {

		elevatorA.setIdleMode(IdleMode.kBrake);

		elevatorA.setInverted(false);
		// elevatorB.setInverted(true);

		elevatorA.enableSoftLimit(SoftLimitDirection.kForward, true);
		elevatorA.enableSoftLimit(SoftLimitDirection.kReverse, true);
		elevatorA.setSoftLimit(SoftLimitDirection.kForward, 22);
		elevatorA.setSoftLimit(SoftLimitDirection.kReverse, 0);
		// elevatorB.follow(elevatorA);

		// elevatorA.burnFlash();
		// elevatorB.burnFlash();

		controller.setTolerance(3);
	}

	public void setState(ElevatorState state) {
		elevatorState = state;
	}

	public ElevatorState getState() {
		return elevatorState;
	}

	public void setElevatorOpenLoop(double percent) {
		elevatorA.set(percent);
	}

	public void setElevatorClosedLoop() {
		@SuppressWarnings("unused")
		double output = 0;
		double feedForward = 0;
		if (encoder.getPosition() < 11){
			feedForward = Constants.Elevator.LOW_ELEVATOR_KS;
		}

		else{
			feedForward = Constants.Elevator.HIGH_ELEVATOR_KS;
		}
	output = controller.calculate(encoder.getPosition(), setpointElevator) + feedForward;
		elevatorA.set(MathUtil.clamp(output, -.2, .7));
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs((setpointElevator - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Current Draw", elevatorA.getOutputCurrent());
		SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
		SmartDashboard.putString("Elevator State", String.valueOf(getState()));
		SmartDashboard.putNumber("Elevator Output", elevatorA.get());
		SmartDashboard.putNumber("Setpoint Elevator", getSetpoint());
		switch (elevatorState) {
			case IDLE:
				setSetpoint(-.9);
				setElevatorClosedLoop();
				break;
			case MANUAL:
				setElevatorOpenLoop(-1 * RobotContainer.operatorPad.getLeftY());//intuitive
				break;
			case POSITION:
				setElevatorClosedLoop();
				break;
			case AMPING:
				setSetpoint(30);
				setElevatorClosedLoop();
			case CLIMBING:

				break;
		}
	}
}
