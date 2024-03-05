// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */
    double setpointClimb = 0;

    public enum ClimbState {
        IDLE,
        POSITION,
        MANUAL,
        TEST
    }

    ClimbState climbState = ClimbState.MANUAL;

    CANSparkMax leftClimber = new CANSparkMax(Constants.Ports.LEFT_CLIMB, MotorType.kBrushless);
    CANSparkMax rightClimber = new CANSparkMax(Constants.Ports.RIGHT_CLIMB, MotorType.kBrushless);

    SparkPIDController leftClimbController = leftClimber.getPIDController();
    SparkPIDController rightClimbController = rightClimber.getPIDController();

    public RelativeEncoder encoderLeft = leftClimber.getEncoder();
    public RelativeEncoder encoderRight = rightClimber.getEncoder();

    public Climb() {
        leftClimber.restoreFactoryDefaults();
        rightClimber.restoreFactoryDefaults();

        leftClimber.setIdleMode(IdleMode.kCoast);
        rightClimber.setIdleMode(IdleMode.kCoast);

        leftClimbController.setP(Constants.Climb.CLIMB_KP);
        leftClimbController.setI(Constants.Climb.CLIMB_KI);
        leftClimbController.setD(Constants.Climb.CLIMB_KD);

        rightClimbController.setP(Constants.Climb.CLIMB_KP);
        rightClimbController.setI(Constants.Climb.CLIMB_KI);
        rightClimbController.setD(Constants.Climb.CLIMB_KD);


        leftClimber.setSoftLimit(SoftLimitDirection.kForward, Constants.Climb.FORWARD_SOFT_LIMIT);
        leftClimber.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climb.REVERSE_SOFT_LIMIT);

        rightClimber.setSoftLimit(SoftLimitDirection.kForward, Constants.Climb.FORWARD_SOFT_LIMIT);
        rightClimber.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climb.REVERSE_SOFT_LIMIT);

    }
    public ClimbState getState(){
        return climbState;
    }

    public void setState(ClimbState state){
        climbState = state;
    }

    public double getSetpoint() {
        return setpointClimb;
    }

    public void setSetpoint(double setpoint) {
        setpointClimb = setpoint;
    }

    public double getLeftClimbPosition() {
        return encoderLeft.getPosition();
    }

    public double getRightClimbPosition() {
        return encoderRight.getPosition();
    }

    public void setLeftClimbPosition(double setpoint) {
        leftClimbController.setFF(Constants.Climb.CLIMB_KFF);
        leftClimbController.setReference(setpoint, ControlType.kPosition);
    }

    public void setRightClimbPosition(double setpoint) {
        rightClimbController.setFF(Constants.Climb.CLIMB_KFF);
        rightClimbController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (climbState) {
            case IDLE:

                break;
            case POSITION:

                break;
            case MANUAL:
                while (RobotContainer.operatorPad.getL1Button()) {
                    leftClimber.set(.5);
                    // setLeftClimbPosition(getLeftClimbPosition() + 5);
                }
                while (RobotContainer.operatorPad.getR1Button()) {
                    rightClimber.set(.5);
                    // setLeftClimbPosition(getRightClimbPosition() + 5);
                }
                while (RobotContainer.operatorPad.getL2Button()) {
                    leftClimber.set(-.5);
                    // setLeftClimbPosition(getLeftClimbPosition() - 5);
                }
                while (RobotContainer.operatorPad.getR2Button()) {
                    rightClimber.set(-.5);
                    // setLeftClimbPosition(getRightClimbPosition() - 5);
                }
                break;
            case TEST:
                System.out.println(encoderLeft.getPosition());
                System.out.println(encoderRight.getPosition());
        }

    }
}
