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
    public static double speakerHeight = 2.10;
    static double targetYaw;

    public static double rotation = 0;
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
                desiredSpeakerTag = blueSpeakerTag;
            } else {
                desiredSpeakerTag = redSpeakerTag;
            }
        }
        return Math.abs(Math.atan((RobotContainer.drivetrain.getState().Pose.getY() - desiredSpeakerTag.getY()) 
       / (RobotContainer.drivetrain.getState().Pose.getX() - desiredSpeakerTag.getX())) -
        RobotContainer.drivetrain.getState().Pose.getRotation().getRadians()) < .1;
    }

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

    public static double 
    getRotationToSpeaker() {
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
            System.out.println(targetYaw);
            System.out.println(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians());
            rotation = 1.5 * targetAlignment
                    .calculate(RobotContainer.drivetrain.getState().Pose.getRotation().getRadians(), targetYaw);
        } else {
            rotation = -RobotContainer.driverPad.getRightX() * Constants.Drive.MaxAngularRate;
        }
        return rotation;
 
    }

    @Override
    public void periodic() {
        System.out.println(isAtRotationSetpoint());

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                desiredAllianceID = 7;
            } else {
                desiredAllianceID = 4;
            }
        }
        
           photonPoseEstimatorShoot.update();
        //    photonPoseEstimatorShoot2.update();

        SmartDashboard.putNumber("Wrist Target Pos", RobotContainer.wrist.getCalculatedPosition());
        SmartDashboard.putNumber("Estimated Distance To Robot By Drivetrain", getDistanceToSpeaker());
        SmartDashboard.putString("Current Robot Pose", RobotContainer.drivetrain.getState().Pose.toString());
        SmartDashboard.putNumber("Rotation", getRotationToSpeaker());

        // This method will be called once per scheduler
        switch (visionState) {
            case DRIVING:
                break;
            case INTAKING:
                break;
            case SHOOTING:
                break;
        }
    }
}
