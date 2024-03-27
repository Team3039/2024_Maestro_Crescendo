// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.InterpolatingDouble;
import frc.util.InterpolatingTreeMap;
import frc.util.Vector2;

public class Vision extends SubsystemBase {
    public static InterpolatingTreeMap<InterpolatingDouble, Vector2> shootingMap;

    // Pose2d previoiusEstimatedPose2d;

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    static Pose3d blueSpeakerTag = aprilTagFieldLayout.getTagPose(7).get();
    static Pose3d redSpeakerTag = aprilTagFieldLayout.getTagPose(4).get();
    static Pose3d desiredSpeakerTag;

    public enum VisionState {
        DRIVING,
        INTAKING,
        SHOOTING
    }

    VisionState visionState = VisionState.DRIVING;

    double setpointWrist;
    double setpointShooter;
    static double distance = 0;
    public static double speakerHeight = 2.027428;
    static double targetYaw;

    public static double rotation;
    int indexID;
    double desiredAllianceID;

    public static PIDController targetAlignment = new PIDController(.06, 0, 0.0035);

    public PhotonCamera shootingCamera = new PhotonCamera("Shooter Cam");
    PhotonPipelineResult resultShoot;

    public PhotonPoseEstimator photonPoseEstimatorShoot = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shootingCamera,
            Constants.Vision.shootCameraToRobot);
    
    /** Creates a new Vision. */
    public Vision() {
        shootingCamera.setDriverMode(false);
        setState(VisionState.DRIVING);
    }

    public VisionState getState() {
        return visionState;
    }

    public void setState(VisionState state) {
        visionState = state;
    }

    public PhotonPipelineResult getCameraResult(PhotonCamera camera, PhotonPipelineResult result) {
        result = camera.getLatestResult();
        return result;
    }

    // public Translation3d getMultiTagResult(PhotonCamera camera) {
    // if (camera.getLatestResult() != null) {
    // var result = camera.getLatestResult();

    // if (result.getMultiTagResult().estimatedPose.isPresent) {
    // Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
    // Translation3d pose = new Translation3d(fieldToCamera.getX(),
    // fieldToCamera.getY(),
    // fieldToCamera.getZ());
    // return pose;
    // }
    // }

    // return null;
    // }

    

    public static double getDistanceToSpeaker() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                desiredSpeakerTag = blueSpeakerTag;
            } else {
                desiredSpeakerTag = redSpeakerTag;
            }
        }
        double SpeakerX = desiredSpeakerTag.getX();

        double distanceXToSpeaker = SpeakerX - RobotContainer.drivetrain.getState().Pose.getX();

        double SpeakerY = desiredSpeakerTag.getY();

        double distanceYToSpeaker = SpeakerY - RobotContainer.drivetrain.getState().Pose.getY();

        distance = Math.hypot(distanceXToSpeaker, distanceYToSpeaker);
        return distance;
    }

    public static double getRotationToSpeaker() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                desiredSpeakerTag = blueSpeakerTag;
            } else {
                desiredSpeakerTag = redSpeakerTag;
            }
        }
        targetYaw = Math.atan((RobotContainer.drivetrain.getState().Pose.getY() -
                desiredSpeakerTag.getY()) /
                (RobotContainer.drivetrain.getState().Pose.getX() - desiredSpeakerTag.getX()));

        if (RobotContainer.driverPad.getCircleButton() == true) {
            rotation = 1.5 * targetAlignment
                    .calculate(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians(), targetYaw);
        } else {
            rotation = -RobotContainer.driverPad.getRightX() * Constants.Drive.MaxAngularRate;
        }
        return rotation;
    }
   

    @Override
    public void periodic() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                desiredAllianceID = 7;
            } else {
                desiredAllianceID = 4;
            }
        }
        resultShoot = shootingCamera.getLatestResult();
        PhotonPipelineResult shootingResult = shootingCamera.getLatestResult();
        
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(
                4 * Math.pow(getDistanceToSpeaker(), 2),
                4 * Math.pow(getDistanceToSpeaker(), 2),
                100));

        if (shootingResult != null) {
           photonPoseEstimatorShoot.update();
        }

        SmartDashboard.putNumber("Wrist Target Pos", RobotContainer.wrist.getCalculatedPosition());
        SmartDashboard.putNumber("Estimated Distance To Robot By Drivetrain", getDistanceToSpeaker());
        SmartDashboard.putString("Current Robot Pose", RobotContainer.drivetrain.getState().Pose.toString());
        SmartDashboard.putNumber("Rotation", getRotationToSpeaker());

        // This method will be called once per scheduler
        switch (visionState) {
            case DRIVING:
                if (resultShoot.getMultiTagResult().estimatedPose.isPresent) {
                Transform3d fieldToCamera = resultShoot.getMultiTagResult().estimatedPose.best;
                System.out.println(fieldToCamera);
                }
                break;
            case INTAKING:
                break;
            case SHOOTING:
                break;
        }
    }
}
