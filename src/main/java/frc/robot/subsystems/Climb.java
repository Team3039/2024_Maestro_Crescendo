// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */

    public enum ClimbState {
        IDLE,
        POSITION,
        MANUAL
    }

    ClimbState climbState = ClimbState.MANUAL;

    TalonFX leftClimber = new TalonFX(Constants.Ports.LEFT_CLIMB);
    TalonFX rightClimber = new TalonFX(Constants.Ports.RIGHT_CLIMB);

    public Climb() {
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);
    }

    public ClimbState getState() {
        return climbState;
    }

    public void setState(ClimbState state) {
        climbState = state;
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
                if (RobotContainer.operatorPad.getL1Button()) {
                    leftClimber.set(.1);
                    // setLeftClimbPosition(getLeftClimbPosition() + 5);
                }
                else if(RobotContainer.operatorPad.getL2Button()){
                    leftClimber.set(-.1);
                }
               else  {
                    leftClimber.set(0);
                    // setLeftClimbPosition(getRightClimbPosition() + 5);
                } 
                 if(RobotContainer.operatorPad.getR1Button()){
                    rightClimber.set(.1);
                }
                else if (RobotContainer.operatorPad.getR2Button()) {
                    rightClimber.set(-.1);
                    // setLeftClimbPosition(getLeftClimbPosition() - 5);
                } else {
                    rightClimber.set(0);
                }
              
                break;
        }

    }
}
