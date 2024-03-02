// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.Optional;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.Vector2;

public class Vision extends SubsystemBase {
  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shootingMap;

  Pose2d previoiusEstimatedPose2d;


    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

public static final record VisionMeasurement (Pose2d measure, double latency, Matrix<N3, N1> stdDevs) {}

    private final Matrix<N3, N1> kDefaultStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  public enum VisionState {
    DRIVING,
    INTAKING,
    SHOOTING
  }

  VisionState visionState = VisionState.DRIVING;
   
   public PhotonCamera shootCamera = new PhotonCamera("ShootCamera");
   public PhotonPoseEstimator photonPoseEstimatorshoot = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      shootCamera,
      Constants.Vision.shootCameraToRobot);

   public PhotonCamera intakeCamera = new PhotonCamera("IntakeCamera");
    PhotonPoseEstimator photonPoseEstimatorIntake = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      intakeCamera,
      Constants.Vision.intakeCameraToRobot);   
      
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonPoseEstimator, Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator == null) {
        // The field layout failed to load, so we cannot estimate poses.
        return Optional.empty(); 
    }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }


  public PhotonPipelineResult resultShoot;
  // boolean hasShootTargets = false;

  // public PhotonPipelineResult resultIntake;
  // boolean hasIntakeTargets = resultIntake.hasTargets();

  public PhotonTrackedTarget targetShoot;
  // public PhotonTrackedTarget targetIntake;


  /** Creates a new Vision. */
  public Vision() {

  shootingMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

  shootingMap.put(new InterpolatingDouble(Double.valueOf(-13.83)), new Vector2(3425, 0));
    shootCamera.setDriverMode(false);
    // intakeCamera.setDriverMode(false);
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

  // public void recieveIntakeTarget() {
  //   if (resultIntake != null) {
  //     if (resultIntake.hasTargets()) {
  //       targetIntake = resultIntake.getBestTarget();
  //     }
  //   }
  // }
  
  /** @return The (field-relative) X distance from the target */
  public double getX() {
    if (visionState.equals(VisionState.DRIVING)) {
      if (resultShoot.hasTargets()) {
        return (resultShoot.getMultiTagResult().estimatedPose.best.getX());
      }
    }
    return 0;
  }
  public double getY() {
    if (visionState.equals(VisionState.DRIVING)) {
      if (resultShoot.hasTargets()) {
        return (resultShoot.getMultiTagResult().estimatedPose.best.getY());
      }
    }
    return 0;
  }
    
  
  @Override
  public void periodic() {
    resultShoot = shootCamera.getLatestResult();
    // resultIntake = IntakeCamera.getLatestResult();

    // This method will be called once per scheduler 
    // getCameraResult(intakeCamera, resultIntake);
    
    // System.out.println(intakeCamera.getPipelineIndex());
    // System.out.println(intakeCamera.getName());
    // System.out.println(intakeCamera.getDriverMode());
    // System.out.println(intakeCamera.getLatestResult());

    // System.out.println(shootCamera.getPipelineIndex());
    // System.out.println(shootCamera.getName());
    System.out.println(shootCamera.getLatestResult().hasTargets());

    switch (visionState) {

      case DRIVING:
        // System.out.println(photonPoseEstimatorIntake.update());
        // getCameraResult(intakeCamera, resultIntake);
        // if(resultIntake.hasTargets()){
        //   if(resultIntake.getMultiTagResult().estimatedPose.isPresent){
        //   System.out.println("true");}
      //     //  Transform3d fieldToCamera = resultIntake.getMultiTagResult().estimatedPose.best;
      //     //  System.out.println("X" + resultIntake.getMultiTagResult().estimatedPose.best.getX());
          //  System.out.println("Y" + resultIntake.getMultiTagResult().estimatedPose.best.getY());
      // };
        // shootCamera.setDriverMode(false);
        // System.out.println(intakeCamera.getName());
        // intakeCamera.setDriverMode(false);
        recieveShootTarget();
        System.out.println(getX());
        System.out.println(getY());

        // System.out.println(shootCamera.getLatestResult().getLatencyMillis());
        // System.out.println(intakeCamera.getLatestResult().getLatencyMillis());
        break;

      case INTAKING:
      //  shootCamera.setDriverMode(true);
      //   getCameraResult(shootCamera, resultShoot);
      //   getCameraResult(intakeCamera, resultIntake);
      //   intakeCamera.setPipelineIndex(2);

      //   if (resultIntake.hasTargets()) {
      //   }
        break;

      case SHOOTING:
      // shootCamera.setDriverMode(false);
      // intakeCamera.setDriverMode(false);
      // intakeCamera.setPipelineIndex(1);
      // recieveShootTarget();
      // recieveIntakeTarget();
        break;

    }
  }
}
