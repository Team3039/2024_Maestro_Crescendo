// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Pose3d SpeakerCenterBlue = new Pose3d(0.2167, 5.549, 2.12, new Rotation3d());
    public static final Pose3d SpeakerCenterRed = new Pose3d(16.3, 5.549, 2.12, new Rotation3d());
    static Pose3d desiredSpeakerPose;

    public enum VisionState {
        DRIVING,
        ROTATING
    }

    VisionState visionState = VisionState.DRIVING;

    double setpointWrist;
    double setpointShooter;
    static double distance = 0;
    public static double speakerHeight = 2.02; //tune this value
    static double targetYaw;

    public static double rotation = 0;
    public static boolean shouldRotateToSpeaker = false;
    int indexID;
    double desiredAllianceID;

    public static PIDController targetAlignment = new PIDController(6, 0, 0.00);

    public PhotonCamera shootingCamera = new PhotonCamera("Shooter Cam");
    // public PhotonCamera shootingCamera2 = new PhotonCamera("Shoot Cam");

    PhotonPipelineResult resultShoot;
    public static final Transform3d shootCameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    // public static final Transform3d shoot2CameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));


    public PhotonPoseEstimator photonPoseEstimatorShoot = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            shootingCamera,
            shootCameraToRobot);

    //  public PhotonPoseEstimator photonPoseEstimatorShoot2 = new PhotonPoseEstimator(aprilTagFieldLayout,
    //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //         shootingCamera,
    //         shoot2CameraToRobot);
    
    /** Creates a new Vision. */
    public Vision() {
        shootingCamera.setDriverMode(false);
        // shootingCamer2.setDriverMode(false);
        setState(VisionState.DRIVING);
    }

    public VisionState getState() {
        return visionState;
    }

    public void setState(VisionState state) {
        visionState = state;
    }
    
    public boolean isAtRotationSetpoint(){

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                desiredSpeakerPose = SpeakerCenterBlue;
            } else {
                desiredSpeakerPose = SpeakerCenterRed;
            }
        }

        return Math.abs(Math.atan((RobotContainer.drivetrain.getState().Pose.getY() - desiredSpeakerPose.getY()) 
       / (RobotContainer.drivetrain.getState().Pose.getX() - desiredSpeakerPose.getX())) -
        RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) < .1;
    }

    public static double getDistanceToSpeaker() {

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                desiredSpeakerPose = SpeakerCenterBlue;
            } else {
                desiredSpeakerPose = SpeakerCenterRed;
            }
        }

        double SpeakerX = desiredSpeakerPose.getX();

        double distanceXToSpeaker = SpeakerX - RobotContainer.drivetrain.getState().Pose.getX();

        double SpeakerY = desiredSpeakerPose.getY();

        double distanceYToSpeaker = SpeakerY - RobotContainer.drivetrain.getState().Pose.getY();

        distance = Math.hypot(distanceXToSpeaker, distanceYToSpeaker);
        return distance;
    }

    public static double getRotationToSpeaker() {

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Blue) {
                desiredSpeakerPose = SpeakerCenterBlue;
            } else {
                desiredSpeakerPose = SpeakerCenterRed;
            }
        }

        targetYaw = Math.atan((RobotContainer.drivetrain.getState().Pose.getY() -
                desiredSpeakerPose.getY()) /
                (RobotContainer.drivetrain.getState().Pose.getX() - desiredSpeakerPose.getX()));

        if (shouldRotateToSpeaker) {
            System.out.println(targetYaw);
            System.out.println(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
            rotation = .5 * targetAlignment
                    .calculate(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians(), targetYaw);
        } else {
            rotation = -RobotContainer.driverPad.getRightX() * Constants.Drive.MaxAngularRate;
        }
        return rotation;
    }

    @Override
    public void periodic() {
        System.out.println(isAtRotationSetpoint());
        
         photonPoseEstimatorShoot.update();
        //    photonPoseEstimatorShoot2.update();

        SmartDashboard.putNumber("Wrist Target Pos", RobotContainer.wrist.getCalculatedPosition());
        SmartDashboard.putNumber("Estimated Distance To Robot By Drivetrain", getDistanceToSpeaker());
        SmartDashboard.putString("Current Robot Pose", RobotContainer.drivetrain.getState().Pose.toString());
        SmartDashboard.putNumber("Rotation", getRotationToSpeaker());

        // This method will be called once per scheduler
        switch (visionState) {
            case DRIVING:
                shouldRotateToSpeaker = false;
                break;
            case ROTATING:
            shouldRotateToSpeaker = true;
                break;
        }
    }
}
