// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.Collection;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

public class Orchestrator extends SubsystemBase {
  /** Creates a new Orchestra. */
  Orchestra orchestra = new Orchestra();

  String[] songs = new String[] { "all-star.chrp", "Imperial-March.chrp", "Cantina-Band.chrp", "Theme-Song.chrp", "Wii-Song.chrp" };
 
  public enum OrchestratorState{
    SILENT,
    MARIOTIME
  }
  OrchestratorState orchestratorState = OrchestratorState.SILENT;

  public Orchestrator(){
     
      for (int module = 0; module < 3; module++) {
       orchestra.addInstrument(RobotContainer.drivetrain.getModule(module).getDriveMotor());
       orchestra.addInstrument(RobotContainer.drivetrain.getModule(module).getSteerMotor());
    }

    orchestra.loadMusic("MarioBros.chrp");
  }
  public void setState (OrchestratorState state){
          orchestratorState = state;
        }
     public OrchestratorState getState() {
          return orchestratorState;
        }     
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(orchestratorState){
      case SILENT:
      if(orchestra.isPlaying()){
      orchestra.pause();
      }
      case MARIOTIME:
      orchestra.play();
    }
  }
}
