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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  public enum VisionState {
    DRIVING,
    INTAKING,
    SHOOTING
  }

  VisionState visionState = VisionState.DRIVING;
   
   public PhotonCamera shootCamera = new PhotonCamera("ShootCamera");
   public PhotonCamera intakeCamera = new PhotonCamera("IntakeCamera");

   AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


   PhotonPoseEstimator photonPoseEstimatorshoot = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.AVERAGE_BEST_TARGETS,
      shootCamera,
      Constants.Vision.shootCameraToRobot);

    PhotonPoseEstimator photonPoseEstimatorIntake = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.AVERAGE_BEST_TARGETS,
      intakeCamera,
      Constants.Vision.intakeCameraToRobot);




  public PhotonPipelineResult resultShoot;
  public PhotonPipelineResult resultIntake;

  public PhotonTrackedTarget targetShoot;
  public PhotonTrackedTarget targetIntake;


  /** Creates a new Vision. */
  public Vision() {
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
  public void getCameraResult(PhotonCamera camera, PhotonPipelineResult result) {
    result = camera.getLatestResult();
  }
  public void recieveShootTarget() {
    if (resultShoot != null) {
      if (resultShoot.hasTargets()) {
        targetShoot = resultShoot.getBestTarget();
      }
    }
  }
  public void recieveIntakeTarget() {
    if (resultIntake != null) {
      if (resultIntake.hasTargets()) {
        targetIntake = resultIntake.getBestTarget();
      }
    }
  }

  /** @return The X (forward/back) distance from the target */
  public double getX() {
    if (visionState.equals(VisionState.DRIVING)) {
      if (resultShoot.hasTargets() && resultIntake.hasTargets() != true) {
        return (targetShoot.getBestCameraToTarget().getX() + targetIntake.getBestCameraToTarget().getX())/2;
      }
    }
    return 0;
  }
    
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(intakeCamera.getPipelineIndex());
    System.out.println(intakeCamera.getName());
    System.out.println(intakeCamera.getDriverMode());
    System.out.println(intakeCamera.getLatestResult().hasTargets());

    System.out.println(shootCamera.getPipelineIndex());
    System.out.println(shootCamera.getName());
    System.out.println(shootCamera.getDriverMode());
    System.out.println(shootCamera.getLatestResult().hasTargets());

    switch (visionState) {

      case DRIVING:
        shootCamera.setDriverMode(false);
        intakeCamera.setDriverMode(true);
        getCameraResult(shootCamera, resultShoot);
        recieveShootTarget();
        break;

      case INTAKING:
       shootCamera.setDriverMode(true);
        getCameraResult(shootCamera, resultShoot);
        getCameraResult(intakeCamera, resultIntake);
        intakeCamera.setPipelineIndex(1);
        recieveShootTarget();
        recieveIntakeTarget();
        if (resultIntake.hasTargets()) {
        }
        break;

      case SHOOTING:
      shootCamera.setDriverMode(false);
      intakeCamera.setDriverMode(false);
      intakeCamera.setPipelineIndex(0);
      recieveShootTarget();
      recieveIntakeTarget();
        break;

    }
  }
}
