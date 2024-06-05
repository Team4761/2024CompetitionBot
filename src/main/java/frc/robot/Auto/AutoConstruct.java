package frc.robot.Auto;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.SwerveTurnTo;
import frc.robot.subsystems.swerve.ZeroGyro;
import frc.robot.subsystems.vision.TurnToTag;
import frc.robot.RobocketsShuffleboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Auto.fullautos.*;

/**
 * <p> This entire class exists solely not to clutter up Robot.java with auto code that gets written in random spots
 * <p> If you have a new auto choisce to add, do it here
 */
public class AutoConstruct extends SendableChooser<String> {

    // Internal Auto names
    private static final String kDefaultAuto = "Default";
    private static final String kOneMeterLeft = "left1meter";
    private static final String kOneMeterForward = "forward1meter";
    private static final String kPathPlanner1Auto = "pathPlanner1";
    private static final String kPathPlanner2Auto = "pathPlanner2";
    private static final String kAprilDance = "aprilDance";
    private static final String kPathPlanner2Up = "upTwoMeter";
    private static final String kPathPlanner1MockAuto = "pathPlanner1MeterForward";
    private static final String kPathPlanner2MeterSpin = "2meterspin";
    private static final String prepShooterAuto = "prepshooterauto";
    private static final String kShootAuto = "shootAuto";
    private static final String kMessEmUpAutoLeft = "messEmUpAutoLeft";
    private static final String kMessEmUpAutoRight = "messEmUpAutoRight";
    private static final String kLeftDiagonalOneNote = "leftDiagonalOneNote";
    private static final String kRightDiagonalOneNote = "rightDiagonalOneNote";
    private static final String kLeftDiagonalTwoNote = "leftDiagonalTwoNote";
    private static final String kRightDiagonalTwoNote = "rightDiagonalTwoNote";
    private static final String kDoNothing = "doNothingAuto";

    private static final String kOneNoteAuto = "oneNoteAuto";
    private static final String kTwoNoteAuto = "twoNoteAuto";
    private static final String kThreeNoteAuto = "threeNoteAuto";
    private static final String kFourNoteAuto = "fourNoteAuto";
    private static final String kFarMiddleAuto = "middleFarAuto";
    private static final String kCloseMiddleAuto = "middleCloseAuto";

    private static final String kRotateTestAuto = "rotateTestAuto";

    private static AutoConstruct autoSelector;
    private static String m_autoSelected;

    /**
     * <p> Sends an  object to Smart Dashboard
     * <p> This is something that is normally done inside Robot.java
     * <p> A default Robot.java will have this code below written directly into autoInit()
     */
    public AutoConstruct() {
        // Construct our Auto Options
        setDefaultOption("Default Auto", kDefaultAuto);
        addOption("Do Nothing Auto", kDoNothing);

        // First parameter: The name of the auto displayed on SmartDashboard
        // Second parameter: The internal name of the auto
        //addOption("Do Nothing", "");
        //addOption("One Meter Left", kOneMeterLeft);
        //addOption("One Meter Forward", kOneMeterForward);
        //addOption("Prep Shooter Auto", prepShooterAuto);
        // addOption("PathPlanner 1 Meter Forward Auto", kPathPlanner1Auto);
        // addOption("PathPlanner 1 Meter Forward Path", kPathPlanner1MockAuto);
        // addOption("PathPlanner Test 2", kPathPlanner2Auto);
        // addOption("April Tag Dance", kAprilDance);
        // addOption("PathPlanner 2 Meter Up", kPathPlanner2Up);
        // addOption("PathPlanner 2 Meter Spin", kPathPlanner2MeterSpin);

        //addOption("Rotate Test", kRotateTestAuto);

        addOption("Shoot Auto", kShootAuto);

        addOption("One Note Auto", kOneNoteAuto);
        addOption("Two Note Auto", kTwoNoteAuto);
        addOption("Three Note Auto", kThreeNoteAuto);
        addOption("Four Note Auto", kFourNoteAuto);
        addOption("Two Note Unintrusive", kFarMiddleAuto);
        addOption("Two Note Squeeze", kCloseMiddleAuto);

        addOption("turn to aptil test", kAprilDance);

        addOption("Mess Em Up Auto Left", kMessEmUpAutoLeft);
        addOption("Mess Em Up Auto Right", kMessEmUpAutoRight);
        addOption("Diagonal One Note Auto Left", kLeftDiagonalOneNote);
        addOption("Diagonal One Note Auto Right", kRightDiagonalOneNote);
        addOption("Diagonal Two Note Auto Left", kLeftDiagonalTwoNote);
        addOption("Diagonal Two Note Auto Right", kRightDiagonalTwoNote);

        autoSelector = this;
    }

    /**
     * <p> Sends the selected auto command to the CommandScheduler
     * <p> The auto command is a class that extends the Command Class
     * <p> This means that the auto command has a built in initialize(), execute(), end(), stop(), and interrupted() to work with
     * <p> Once the auto command decides it is finished for whatever reason, the command scheduler releases the command and runs the next one in the stack
     * <p> It should be noted that everytime a new auto is added, we must write the code ourselves
     * @param map Some commands require that you pass in a subsystem. Fortunately all these subsystems can be found in RobotMap, so we will just pass that in
     */
    public static void scheduleSelectedCommand(RobotMap map) {

        m_autoSelected = autoSelector.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // System.out.println("Current: " + pose);
            SmartDashboard.putNumber("PathPlanner X", pose.getX());
            SmartDashboard.putNumber("PathPlanner Y", pose.getY());
            SmartDashboard.putNumber("PathPlanner Rot", pose.getRotation().getDegrees());
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // System.out.println("Target: " + pose);
            SmartDashboard.putNumber("PathPlanner Target X", pose.getX());
            SmartDashboard.putNumber("PathPlanner Target Y", pose.getY());
            SmartDashboard.putNumber("PathPlanner Target Rot", pose.getRotation().getDegrees());
        });

        
        Command scheduledCommand = null;
        switch (m_autoSelected) {
            case kDefaultAuto: // do nothing
                scheduledCommand = new ZeroGyro();
                break;
            //case kOneMeterLeft:
            //    scheduledCommand = new SwerveGoCartesianF(map.swerve, new Translation2d(0, 1));
            //    break;
            //case kOneMeterForward:
            //    scheduledCommand = new SwerveGoCartesianF(map.swerve, new Translation2d(1, 0));
            //    break;
            // case prepShooterAuto:
            //     scheduledCommand = new PrepShooterAuto();
            //     break;
            case kShootAuto:
                scheduledCommand = new ShootAuto();
                break;
            //case kRotateTestAuto:
            //    scheduledCommand = new SwerveTurnTo(map.swerve, new Rotation2d(Math.PI/2));
            //    break;
            // case kPathPlanner1Auto:
            //     scheduledCommand = new PathPlannerAuto("1 Meter Auto").andThen(new PrintCommand("Path Finished!"));
            // break;
            // case kPathPlanner1MockAuto:
            //     scheduledCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile("1 Meter Path"))/*.andThen(new SwerveDriveStop())*/;
            // break;
            // case kPathPlanner2Up:
            //     scheduledCommand = new PathPlannerAuto("2 Meter Up")/*.andThen(new SwerveDriveStop())*/;
            // break;
            // case kPathPlanner2Auto:
            //     scheduledCommand = new PathPlannerAuto("Simple Swerve Auto");
            // break;
            //case kAprilDance:
                //scheduledCommand = new TurnToTag(Robot.getMap().swerve, Robot.getMap().vision);
                //break;
            // case kPathPlanner2MeterSpin:
            //     scheduledCommand = new PathPlannerAuto("2 Meter Spin");
            // break;
            case kOneNoteAuto:
                scheduledCommand = new OneNoteAuto();
                break;
            case kTwoNoteAuto:
                scheduledCommand = new TwoNoteAuto();
                break;
            case kMessEmUpAutoLeft:
                scheduledCommand = new MessEmUpAuto(true);
                break;
            case kMessEmUpAutoRight:
                scheduledCommand = new MessEmUpAuto(false);
                break;
            case kLeftDiagonalOneNote:
                scheduledCommand = new DiagonalOneNoteAuto(true);
                break;
            case kRightDiagonalOneNote:
                scheduledCommand = new DiagonalOneNoteAuto(false);
                break;
            case kLeftDiagonalTwoNote:
                scheduledCommand = new DiagonalTwoNoteAuto(true);
                break;
            case kRightDiagonalTwoNote:
                scheduledCommand = new DiagonalTwoNoteAuto(false);
                break;

            case kDoNothing:
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