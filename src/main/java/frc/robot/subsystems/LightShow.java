// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;

public class LightShow extends SubsystemBase {
    /** Creates a new Orchestra. */
    CANdle light = new CANdle(Constants.Ports.CANdleID);


    public enum LightShowState {
        SILENT
    }

    LightShowState lightShowState = LightShowState.SILENT;

    public LightShow() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = .8; // dim the LEDs to 8/10 brightness
        light.configAllSettings(config);

        // for (int module = 0; module < 3; module++) {
        //     show.addInstrument(RobotContainer.drivetrain.getModule(module).getDriveMotor());
        //     show.addInstrument(RobotContainer.drivetrain.getModule(module).getSteerMotor());
        // }
    }

    public void setState(LightShowState state) {
        lightShowState = state;
    }

    public LightShowState getState() {
        return lightShowState;
    }

    // has Note
    public void lightOrange() {
        // set brightness
        light.configBrightnessScalar(1);
        // set color
        light.setLEDs(255, 85, 0);
    }

    public void lightBlue() {
        // set brightness
        light.configBrightnessScalar(1);
        // set color
        light.setLEDs(0, 0, 255);
    }

    // Intaking
    public void lightGreen() {
        // set brightness
        light.configBrightnessScalar(1);
        // set color
        light.setLEDs(0, 255, 0);
    }

    public void lightRed() {
        // set brightness
        light.configBrightnessScalar(1);
        // set color
        light.setLEDs(255, 0, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        switch (lightShowState) {
            case SILENT:
              
               if (RobotContainer.intake.intakeState == IntakeState.RELEASE) {
                    lightRed();
                } else if (RobotContainer.indexer.beamBreak.get()) {
                    lightOrange();
                } else if (RobotContainer.intake.intakeState == IntakeState.INTAKING) {
                    lightGreen();
                } else{
                    light.setLEDs(0, 0, 0);
                }
                break;
        }
    }
}
