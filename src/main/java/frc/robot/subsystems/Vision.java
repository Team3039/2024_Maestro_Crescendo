// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  public enum VisionState {
    DRIVING,
    INTAKING
  }

  VisionState visionState = VisionState.DRIVING;

   public PhotonCamera intakeCamera = new PhotonCamera("IntakeCamera");
   public PhotonCamera shootCamera = new PhotonCamera("ShootCamera");
   public PhotonCamera driveCamera = new PhotonCamera("DriveCamera");

   AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


   PhotonPoseEstimator photonPoseEstimatorA = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      shootCamera,
      Constants.Vision.shootCameraToRobot);

    PhotonPoseEstimator photonPoseEstimatorB = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      intakeCamera,
      Constants.Vision.intakeCameraToRobot);




  public PhotonPipelineResult resultA;
  public PhotonPipelineResult resultB;

  public PhotonTrackedTarget targetA;
  public PhotonTrackedTarget targetB;


  /** Creates a new Vision. */
  public Vision() {
    driveCamera.setDriverMode(true);
    shootCamera.setDriverMode(true);
    intakeCamera.setDriverMode(true);

    setState(VisionState.DRIVING);
  }

public VisionState getState() {
    return visionState;
  }

  public void setState(VisionState state) {
    visionState = state;
  }
  public void getCameraResult() {
    resultA = shootCamera.getLatestResult();
    if(intakeCamera.getPipelineIndex() == 1){
      resultB = intakeCamera.getLatestResult();
    }
  }
  public void recieveShootTarget() {
    if (resultA != null) {
      if (resultA.hasTargets()) {
        targetA = resultA.getBestTarget();
      }
    }
  }
  public void recieveIntakeTarget() {
    if (resultB != null) {
      if (resultB.hasTargets()) {
        targetB = resultB.getBestTarget();
      }
    }
  }

  /** @return The X (forward/back) distance from the target */
  public double getX() {
    if (visionState.equals(VisionState.DRIVING)) {
      if (resultA.hasTargets() && resultB.hasTargets()) {
        return (targetA.getBestCameraToTarget().getX() + targetB.getBestCameraToTarget().getX())/2;
      }
      
    }
    return 0;
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (visionState) {
      case DRIVING:
        shootCamera.setDriverMode(false);
        driveCamera.setDriverMode(true);
        intakeCamera.setDriverMode(true);
        break;
      case INTAKING:
       shootCamera.setDriverMode(true);
       driveCamera.setDriverMode(true);
        getCameraResult();
        recieveShootTarget();
        recieveIntakeTarget();
        if (resultA.hasTargets()) {
          
        }
    }
  }
}
