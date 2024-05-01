package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Drive extends SwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Drive(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        configNeutralMode(NeutralModeValue.Brake);
    }

    public Drive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        configNeutralMode(NeutralModeValue.Brake);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(3.0, 0, 0), // Translation constants
                new PIDConstants(3.0, 0, 0), // Rotation constants
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                pathFollowerConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue, // Change this if
                   // the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        super.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    @SuppressWarnings("deprecation")
    public void periodic() {
        if (!RobotState.isAutonomous()) {
            RobotContainer.vision.leftCamera.getLatestResult();
            setVisionMeasurementStdDevs(VecBuilder.fill(
                    3 * Math.pow(Vision.getDistanceToSpeaker(), 1.6),
                    3 * Math.pow(Vision.getDistanceToSpeaker(), 1.6),
                    100));
            if (RobotContainer.vision.leftCamera.getLatestResult().hasTargets()) {
                // // System.out.println("has targets");
                if(Vision.getMultiTagResult(RobotContainer.vision.leftCamera)!=null ){
                    Translation3d pose = Vision.getMultiTagResult(RobotContainer.vision.leftCamera);
                    double time = Timer.getFPGATimestamp();

                    if(pose !=null){
                    Pose2d usablePose = new Pose2d(pose.toTranslation2d(), RobotContainer.drivetrain.getState().Pose.getRotation());
                    addVisionMeasurement(usablePose, time);
                }


                // if (!RobotContainer.vision.leftCamPoseEstimator.update().isEmpty()) {
                //     System.out.println(RobotContainer.vision.leftCamPoseEstimator.update());
                    // Pose3d pose = RobotContainer.vision.leftCamPoseEstimator.update().get().estimatedPose;
                    // double time = RobotContainer.vision.leftCamPoseEstimator.update().get().timestampSeconds;

                    // if (pose != null) {
                    //     System.out.println("has pose");
                    //     System.out.println(pose);

                    //     Pose2d usablePose = pose.toPose2d();
                    //     addVisionMeasurement(usablePose, time);
                    // }
                }
            }

            // if( RobotContainer.vision.shootingCamera2.hasTargets()){
            // if(!RobotContainer.vision.rightCamPoseEstimator.update().isEmpty()){
            // Pose3d pose =
            // RobotContainer.vision.rightCamPoseEstimator.update().get().estimatedPose;
            // double time =
            // RobotContainer.vision.rightCamPoseEstimator.update().get().timestampSeconds;

            // if (pose != null &&
            // RobotContainer.vision.shootingCamera2.getLatestResult().getBestTarget().getPoseAmbiguity()
            // < .3){
            // Pose2d usablePose = pose.toPose2d();
            // addVisionMeasurement(usablePose, time);
            // }
            // }
            // }

        }

    };
}
