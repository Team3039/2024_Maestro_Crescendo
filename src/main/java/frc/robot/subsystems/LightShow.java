// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeState;

public class LightShow extends SubsystemBase {
    /** Creates a new Orchestra. */
    Orchestra show = new Orchestra();
    CANdle light = new CANdle(Constants.Ports.CANdleID);

    int song = 0;

    String[] songs = {
            "HaloConcert.chrp",
            "Lifelight.chrp",
            "WiiSmashRemix.chrp",
            "TeenTitans.chrp",
            "HaloConcert.chrp"
    };

    public enum LightShowState {
        SILENT,
        PLAYLIST,
        LIGHTSHOW
    }

    LightShowState lightShowState = LightShowState.SILENT;

    FireAnimation redIDLE = new FireAnimation(.6, .4, 50, .6, .6);
    ColorFlowAnimation blueIDLE = new ColorFlowAnimation(0, 100, 255);

    public LightShow() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = .8; // dim the LEDs to 8/10 brightness
        light.configAllSettings(config);

        for (int module = 0; module < 3; module++) {
            show.addInstrument(RobotContainer.drivetrain.getModule(module).getDriveMotor());
            show.addInstrument(RobotContainer.drivetrain.getModule(module).getSteerMotor());
        }
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
                if (show.isPlaying()) {
                    show.stop();
                }
                if (!RobotState.isTeleop() && !RobotState.isAutonomous()) {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if (alliance.get() == DriverStation.Alliance.Blue) {
                            light.animate(blueIDLE);
                        } else {
                            light.animate(redIDLE);
                        }
                    }
                } else if (RobotContainer.intake.intakeState == IntakeState.RELEASE) {
                    lightRed();
                } else if (RobotContainer.indexer.hasNote && RobotState.isTeleop()) {
                    lightOrange();
                } else if (RobotContainer.intake.intakeState == IntakeState.INTAKING && RobotState.isTeleop()) {
                    lightGreen();
                } else {
                    light.setLEDs(0, 0, 0);
                }
                break;

            case PLAYLIST:
                if (RobotContainer.operatorPad.getPOV() == 90 && song < 5) {
                    song += 1;
                    show.loadMusic(songs[song]);
                    show.play();
                }
                if (RobotContainer.operatorPad.getPOV() == 270 && song > 0) {
                    song -= 1;
                    show.loadMusic(songs[song]);
                    show.play();
                }
                break;

            case LIGHTSHOW:
                light.animate(redIDLE);
                show.loadMusic("FinalCountdown.chrp");
                break;
        }
    }
}
