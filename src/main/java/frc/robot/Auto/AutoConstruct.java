package frc.robot.Auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.RobotMap;

/**
 * <p> This entire class exists solely not to clutter up Robot.java with auto code that gets written in random spots
 * <p> If you have a new auto choisce to add, do it here
 */
public class AutoConstruct {

    // Internal Auto names
    private static final String kDefaultAuto = "Default";
    private static final String kOneMeterLeft = "left1meter";
    private static final String kPathPlanner1Auto = "pathPlanner1";
    private static final String kPathPlanner2Auto = "pathPlanner2";
    private static final String kAprilDance = "aprilDance";


    private static String m_autoSelected;
    private static final SendableChooser<String> m_chooser = new SendableChooser<>();

    /**
     * <p> Sends an m_chooser object to Smart Dashboard
     * <p> This is something that is normally done inside Robot.java
     * <p> A default Robot.java will have this code below written directly into autoInit()
     */
    public static void sendAutoOptionsToSmartDashboard() {
        // Construct our Auto Options
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);

        // First parameter: The name of the auto displayed on SmartDashboard
        // Second parameter: The internal name of the auto
        m_chooser.addOption("One Meter Left", kOneMeterLeft);
        m_chooser.addOption("PathPlanner 1 Meter Forward", kPathPlanner1Auto);
        m_chooser.addOption("PathPlanner Test 2", kPathPlanner2Auto);
        m_chooser.addOption("April Tag Dance", kAprilDance);

        // Add the chooser to smartdashboard
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
     * <p> Sends the selected auto command to the CommandScheduler
     * <p> The auto command is a class that extends the Command Class
     * <p> This means that the auto command has a built in initialize(), execute(), end(), stop(), and interrupted() to work with
     * <p> Once the auto command decides it is finished for whatever reason, the command scheduler releases the command and runs the next one in the stack
     * <p>
     * <p> It should be noted that everytime a new auto is added, we must write the code ourselves
     * @param map Some commands require that you pass in a subsystem. Fortunately all these subsystems can be found in RobotMap, so we will just pass that in
     */
    public static void scheduleSelectedCommand(RobotMap map) {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);

        Command scheduledCommand = null;
        switch (m_autoSelected) {
            case kDefaultAuto:
                scheduledCommand = new SwerveGoCartesianF(map.swerve, new Translation2d(1, 0));
            break;
            case kOneMeterLeft:
                scheduledCommand = new SwerveGoCartesianF(map.swerve, new Translation2d(0, 1));
            break;
            case kPathPlanner1Auto:
                scheduledCommand = new PathPlannerAuto("1 Meter Auto");
            break;
            case kPathPlanner2Auto:
                scheduledCommand = new PathPlannerAuto("Simple Swerve Auto");
            break;
            case kAprilDance:
                // we do not have code for this yet
            break;
            default:
                // unsure what the default command would be: maybe just ensure nothing is moving?
            break;

        }

        if(scheduledCommand != null)
            CommandScheduler.getInstance().schedule(scheduledCommand);
        else {
            System.out.println("No Autonomous Command running");
        }
        //CommandScheduler.getInstance().schedule(new SwerveGoCartesianF(map.swerve, new Translation2d(20, 20)));
    }
}