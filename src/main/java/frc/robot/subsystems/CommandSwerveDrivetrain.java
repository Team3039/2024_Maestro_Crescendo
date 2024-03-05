package frc.robot.subsystems;

import java.util.function.Supplier;
// import com.choreo.lib;

import com.ctre.phoenix6.configs.jni.ConfigJNI;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // public PathPlannerPath redTrapClosePath = PathPlannerPath.fromPathFile("Red Trap Close");
    // public PathPlannerPath redTrapFar = PathPlannerPath.fromPathFile("Red Trap Far");
    // public PathPlannerPath redAmpPath = PathPlannerPath.fromPathFile("Red Amp");
    double driveBaseRadius = 2.0;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configNeutralMode(NeutralModeValue.Brake);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configNeutralMode(NeutralModeValue.Brake);

    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    // Pathplanner configs and Pathfind Configs

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            4.5, 3.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0),
            new PIDConstants(1, 0, 0), TunerConstants.kSpeedAt12VoltsMps, driveBaseRadius, new ReplanningConfig());

    public void configurePathPlanner() {
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        // Default Rotation Delay, can change in instantiation of MyPathFinder in
        // Robot.java
        double rotationDelay = 0.0;

        // Default PathFind to follow after pathfind
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Do Nothing");

        // @SuppressWarnings("unused")
        // Command pathfindinCommand = AutoBuilder.pathfindThenFollowPath(
        //         path,
        //         constraints,
        //         rotationDelay // Rotation delay distance in meters. This is how far the robot should travel
        // // before attempting to rotate.
        // );

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
        // final PathfindThenFollowPathHolonomic path = new
        // PathfindThenFollowPathHolonomic(PathPlannerPath.fromPathFile("Red Trap
        // Close"), constraints, () -> this.getState().Pose,
        // this::getCurrentRobotChassisSpeeds, (speeds) ->
        // this.setControl(autoRequest.withSpeeds(speeds)), holonomicPathFollowerConfig,
        // ()-> false, this);
      
    }
    // public Command redAmpFPH = new PathfindThenFollowPathHolonomic(redAmpPath,
    // constraints, () -> this.getState().Pose,
    // () -> this.getCurrentRobotChassisSpeeds(),
    // (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), holonomicPathFollowerConfig,
    // () -> {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    // }, this);

    // public Command redTrapClose = new PathfindThenFollowPathHolonomic(redTrapClosePath,
    // constraints, () -> this.getState().Pose,
    // () -> this.getCurrentRobotChassisSpeeds(),
    // (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), holonomicPathFollowerConfig,
    // () -> {
    //     var alliance = DriverStation.getAlliance();
    //     if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //     }
    //     return false;
    // }, this);
    
    PathConstraints getPathConstraints(PathConstraints constraints) {
        return constraints;
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }
}
