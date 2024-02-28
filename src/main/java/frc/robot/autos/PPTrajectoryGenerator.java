package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
// import frc.robot.commands.ActuateIntake;

public class PPTrajectoryGenerator {
    public static PPTrajectoryGenerator INSTANCE = new PPTrajectoryGenerator();

    public static HashMap<String, Command> eventMap;
   
   public PPTrajectoryGenerator(){
    NamedCommands.registerCommands(eventMap);

    // eventMap.put("Intake", new ActuateIntake());
    // eventMap.put("Hood Position", new SetHoodPosition());
    // eventMap.put("Shoot Note", new ShootNote());

   }

}
