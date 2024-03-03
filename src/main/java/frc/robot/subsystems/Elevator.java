// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

	public enum ElevatorState {
		IDLE,
		MANUAL,
		POSITION,
		AMPING
	}

	public ElevatorState elevatorState = ElevatorState.MANUAL;

	public CANSparkMax elevatorA = new CANSparkMax(Constants.Ports.ELEVATOR_A, MotorType.kBrushless);
	public CANSparkMax elevatorB = new CANSparkMax(Constants.Ports.ELEVATOR_B, MotorType.kBrushless);

	public RelativeEncoder encoder = elevatorA.getEncoder();

	public ElevatorFeedforward feedForward = new ElevatorFeedforward(
			Constants.Elevator.ELEVATOR_KS,
			Constants.Elevator.ELEVATOR_KG,
			Constants.Elevator.ELEVATOR_KV);

	private PIDController controller = new PIDController(
			Constants.Elevator.ELEVATOR_KP,
			Constants.Elevator.ELEVATOR_KI,
			Constants.Elevator.ELEVATOR_KD);

	// neo rotations
	public static double setpointElevator = 0;

	public Elevator() {

		elevatorA.setIdleMode(IdleMode.kBrake);

		elevatorA.setInverted(true);
		elevatorB.setInverted(true);

		elevatorA.enableSoftLimit(SoftLimitDirection.kForward, true);
		elevatorA.enableSoftLimit(SoftLimitDirection.kReverse, true);
		elevatorA.setSoftLimit(SoftLimitDirection.kForward, 30);
		elevatorA.setSoftLimit(SoftLimitDirection.kReverse, 0);
		elevatorB.follow(elevatorA);

		elevatorA.burnFlash();
		elevatorB.burnFlash();

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
		output = controller.calculate(encoder.getPosition(), setpointElevator) + Constants.Elevator.ELEVATOR_KS;
		// elevator.set(MathUtil.clamp(output, -.2, .3));
	}

	public static double getSetpoint() {
		return setpointElevator;
	}

	public static void setSetpoint(double setpoint) {
		setpointElevator = setpoint;
	}

	public boolean isAtSetpoint( double tolerance) {
		return Math.abs((setpointElevator - encoder.getPosition())) <= tolerance;
	}

	public double getPosition() {
		return encoder.getPosition();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Encoder", encoder.getPosition());
		// SmartDashboard.putString("Elevator State", String.valueOf(getState()));

		// SmartDashboard.putNumber("Elevator Output", elevator.get());
		System.out.println(encoder.getPosition());
		// System.out.println(elevator.get());
		// System.out.println(isAtSetpoint(false));
		switch (elevatorState) {
			case IDLE:
				setSetpoint(0);
				setElevatorClosedLoop();
				break;
			case MANUAL:
				setElevatorOpenLoop(RobotContainer.operatorPad.getLeftY());
				break;
			case POSITION:
				setElevatorClosedLoop();
				break;
			case AMPING:
				setSetpoint(30);
				setElevatorClosedLoop();
		}
	}
}
