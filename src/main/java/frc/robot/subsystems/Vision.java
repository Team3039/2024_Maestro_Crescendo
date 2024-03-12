// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.InterpolatingDouble;
import frc.util.InterpolatingTreeMap;
import frc.util.Vector2;

public class Vision extends SubsystemBase {
  public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shootingMap;

  Pose2d previoiusEstimatedPose2d;

  public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  static Pose3d blueSpeakerTag = aprilTagFieldLayout.getTagPose(7).get();
  static Pose3d redSpeakerTag = aprilTagFieldLayout.getTagPose(4).get();

  public enum VisionState {
    DRIVING,
    INTAKING,
    SHOOTING
  }
  VisionState visionState = VisionState.DRIVING;

  double setpointWrist;
  double setpointShooter;
  static double distance = 0;

  public PhotonCamera shootLeftCamera = new PhotonCamera("Left Shooter Cam");

  public PhotonPoseEstimator photonPoseEstimatorLeftShoot = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      shootLeftCamera,
      Constants.Vision.shootCameraToRobot);

  public PhotonPipelineResult resultLeftShooter;

  public PhotonTrackedTarget targetLeftShooter;

  public PhotonCamera shootRightCamera = new PhotonCamera("Right Shooter Cam");

  public PhotonPoseEstimator photonPoseEstimatorRightShoot = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      shootRightCamera,
      Constants.Vision.shootCameraToRobot);

  public PhotonPipelineResult resultRightShooter;

  public PhotonTrackedTarget targetRightShooter;

  public PhotonCamera driveCamera = new PhotonCamera("Driver Camera");
  public PhotonPipelineResult resultDriveCamera;
  public PhotonTrackedTarget targetDrive;


  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator photonPoseEstimator,
      Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      // The field layout failed to load, so we cannot estimate poses.
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }


  /** Creates a new Vision. */
  public Vision() {
    shootingMap = new InterpolatingTreeMap<InterpolatingDouble, Vector2>();

    // InterpolatingDouble (Double.valueOf(distanceFromTarget), Vector 2(shooterRPS, wristAngle))
    shootingMap.put(new InterpolatingDouble(Double.valueOf(1)), new Vector2(56,100));
    shootingMap.put(new InterpolatingDouble(Double.valueOf(2)), new Vector2(50, 100));
    shootingMap.put(new InterpolatingDouble(Double.valueOf(3)), new Vector2(45, 100));
    shootingMap.put(new InterpolatingDouble(Double.valueOf(4)), new Vector2(40, 100));
    shootingMap.put(new InterpolatingDouble(Double.valueOf(5)), new Vector2(35, 80));







    RobotContainer.drivetrain.setVisionMeasurementStdDevs(Constants.Vision.kDefaultStdDevs);

    shootLeftCamera.setDriverMode(false);
    shootRightCamera.setDriverMode(false);
    driveCamera.setDriverMode(true);
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
    

  /** @return The (field-relative) X distance from the target */
  public double getX() {
    if (visionState.equals(VisionState.DRIVING)) {
        if (resultLeftShooter != null){
      if (resultLeftShooter.hasTargets()) {
        return (resultLeftShooter.getMultiTagResult().estimatedPose.best.getX());
      }
    }
    }
    return 0;
  }

  public double getY() {
    if (visionState.equals(VisionState.DRIVING)) {
        if (resultLeftShooter != null){
      if (resultLeftShooter.hasTargets()) {
        return (resultLeftShooter.getMultiTagResult().estimatedPose.best.getY());
      }
    }
    }
    return 0;
  }

  public static double getDistanceToSpeaker(){
    var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() ){
        if(alliance.get() == DriverStation.Alliance.Blue){
          double SpeakerX = blueSpeakerTag.getX();
          double distanceXToSpeaker = SpeakerX - RobotContainer.drivetrain.getState().Pose.getX();
          double SpeakerY = blueSpeakerTag.getY();
          double distanceYToSpeaker = SpeakerY - RobotContainer.drivetrain.getState().Pose.getY();
          distance = Math.hypot(distanceXToSpeaker, distanceYToSpeaker);
        }
        else{
            double SpeakerX = redSpeakerTag.getX();
          double distanceXToSpeaker = SpeakerX - RobotContainer.drivetrain.getState().Pose.getX();
          double SpeakerY = redSpeakerTag.getY();
          double distanceYToSpeaker = SpeakerY - RobotContainer.drivetrain.getState().Pose.getY();
          distance = Math.hypot(distanceXToSpeaker, distanceYToSpeaker);
        }}

          return distance;
  }

  @Override
  public void periodic() {

    setpointWrist = shootingMap.getInterpolated(new InterpolatingDouble(Vision.getDistanceToSpeaker())).x;
    setpointShooter = shootingMap.getInterpolated(new InterpolatingDouble(Vision.getDistanceToSpeaker())).y;

    SmartDashboard.putNumber("Shooter Target Velocity", setpointShooter);
    SmartDashboard.putNumber("Wrist Target Pos", setpointWrist);
    SmartDashboard.putNumber("Estimated Distance To Robot By Drivetrain", getDistanceToSpeaker());


    // resultShoot = shootCamera.getLatestResult();
    // resultIntake = IntakeCamera.getLatestResult();

    // This method will be called once per scheduler
    // getCameraResult(intakeCamera, resultIntake);

    // System.out.println(intakeCamera.getPipelineIndex());
    // System.out.println(intakeCamera.getName());
    // System.out.println(intakeCamera.getDriverMode());
    // System.out.println(intakeCamera.getLatestResult());

    // System.out.println(shootCamera.getPipelineIndex());
    // System.out.println(shootCamera.getName());
    // System.out.println(shootCamera.getLatestResult().hasTargets());

    switch (visionState) {

      case DRIVING:
        // System.out.println(photonPoseEstimatorLeftShoot.update());
        // getCameraResult(intakeCamera, resultIntake);
        // if(resultIntake.hasTargets()){
        // if(resultIntake.getMultiTagResult().estimatedPose.isPresent){
        // System.out.println("true");}
        // // Transform3d fieldToCamera =
        // resultIntake.getMultiTagResult().estimatedPose.best;
        // // System.out.println("X" +
        // resultIntake.getMultiTagResult().estimatedPose.best.getX());
        // System.out.println("Y" +
        // resultIntake.getMultiTagResult().estimatedPose.best.getY());
        // };
        // shootCamera.setDriverMode(false);
        // System.out.println(intakeCamera.getName());
        // intakeCamera.setDriverMode(false);
        // recieveShootTarget();
        // System.out.println(getX());
        // System.out.println(getY());

        System.out.println(shootLeftCamera.getLatestResult().getLatencyMillis());
        // System.out.println(intakeCamera.getLatestResult().getLatencyMillis());
        break;

      case INTAKING:
        // shootCamera.setDriverMode(true);
        // getCameraResult(shootCamera, resultShoot);
        // getCameraResult(intakeCamera, resultIntake);
        // intakeCamera.setPipelineIndex(2);

        // if (resultIntake.hasTargets()) {
        // }
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
