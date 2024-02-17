package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.AutoConstruct;
import frc.robot.subsystems.climber.LoosenClimber;
import frc.robot.subsystems.climber.TightenClimber;
import frc.robot.subsystems.intake.FullIntake;
import frc.robot.subsystems.leds.LedChargeUp;
import frc.robot.subsystems.shooter.GetShooterToAngle;
import frc.robot.subsystems.shooter.Shoot;
import frc.robot.subsystems.shooter.ShootAtAngle;
import frc.robot.subsystems.swerve.SwerveDriveStop;
import frc.robot.subsystems.swerve.SwerveGoCartesianF;
import frc.robot.subsystems.swerve.SwerveTurn;
import frc.robot.subsystems.swerve.SwerveTurnTo;

/**
 * <p> This is a handler class for everything that has to do with the shufflboard.
 * <p> For example, all of our settings for speeds can be accessed using the getSettingNum method.
 */
public class RobocketsShuffleboard {

    private static final String DEBUG = "Debug";
    private static final String SETTINGS = "Settings";

    private static RobocketsShuffleboard shuffleboard;

    private ShuffleboardTab debugTab;
    private ShuffleboardTab settingsTab;

    private ShuffleboardLayout settingsShooter;
    private ShuffleboardLayout settingsSwerve;
    private ShuffleboardLayout settingsIntake;
    private ShuffleboardLayout settingsClimber;
    private ShuffleboardLayout settingsAuto;

    private ShuffleboardLayout debugCommands;

    private ArrayList<SimpleWidget> settings;

    /**
     * <p> Constructor for the shuffleboard which initializes the tabs/layouts and adds the settings.
     */
    public RobocketsShuffleboard () {
        shuffleboard = this;    // Make sure that the static (global) shuffleboard being used in the initialized RobocketsShuffleboard

        // Initialize the tabs (pages).
        debugTab = Shuffleboard.getTab(DEBUG);
        settingsTab = Shuffleboard.getTab(SETTINGS);

        // Initialize the entry arrays
        settings = new ArrayList<SimpleWidget>();

        // Initialize the layouts (groups of values)
        settingsShooter = settingsTab.getLayout("Shooter", BuiltInLayouts.kList).withSize(2,6).withPosition(0,0);
        settingsSwerve = settingsTab.getLayout("Swerve", BuiltInLayouts.kList).withSize(2,6).withPosition(2,0);
        settingsIntake = settingsTab.getLayout("Intake", BuiltInLayouts.kList).withSize(2,6).withPosition(4,0);
        settingsClimber = settingsTab.getLayout("Climber", BuiltInLayouts.kList).withSize(2,6).withPosition(6,0);
        settingsAuto = settingsTab.getLayout("Auto", BuiltInLayouts.kList).withSize(2,6).withPosition(8,0);

        debugCommands = debugTab.getLayout("Commands", BuiltInLayouts.kList).withSize(2,6).withPosition(0,0);

        // Populate the layouts
        addSettings();
        addCommands();
    }

    /**
     * <p> Adds any settings to the shuffleboard {Settings} layout, including, but not limited to...
     * <p> Speeds, Autos, LEDs, etc.
     */
    public void addSettings() {
        // Shooter settings
        addShooterSetting("Shooter In Speed", 0.5);
        addShooterSetting("Shooter Out Speed", 0.5);
        addShooterSetting("Shooter Intake Speed", 0.5);
        addShooterSetting("Shooter Outtake Speed", 0.5);

        // Swerve settings
        addSwerveSetting("Movement Speed", 0.5);
        addSwerveSetting("Rotation Speed", 0.5);
        addSwerveSetting("Is Robot Relative", true);

        // Intake settings
        addIntakeSetting("Intake Speed", 0.5);
        addIntakeSetting("Outtake Speed", 0.5);
        addIntakeSetting("Full Intake Setting", 0.5);

        // Climber settings
        addClimberSetting("Climber Speed", 0.5);

        // Auto settings
        shuffleboard.settingsAuto.add("Auto", new AutoConstruct()); // Cannot be found using the getSettingNum function. Must use the AutoConstruct.scheduleSelectedCommand() method.
    }

    /**
     * <p> Adds all the command for testing to the debug tab.
     */
    public void addCommands() {
        // Swerve commands
        // addCommand("Go 1 Meter Forward", new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(1,0)));
        // addCommand("Go 1 Meter Left", new SwerveGoCartesianF(Robot.getMap().swerve, new Translation2d(0,1)));
        // addCommand("Turn 90 Degrees", new SwerveTurn(Robot.getMap().swerve, new Rotation2d(Units.degreesToRadians(90))));
        // addCommand("Turn To 90 Degrees", new SwerveTurnTo(Robot.getMap().swerve, new Rotation2d(Units.degreesToRadians(90))));

        // Shooter commands
        addCommand("Shoot", new Shoot(getSettingNum("Shooter Out Speed")));
        addCommand("Turn Shooter To 45 Degrees", new GetShooterToAngle(Units.degreesToRadians(45)));
        addCommand("Shoot At 45 Degrees", new ShootAtAngle(Units.degreesToRadians(45), getSettingNum("Shooter Out Speed")));
        
        // Intake settings
        addCommand("Full Intake", new FullIntake(getSettingNum("Full Intake Speed"), Robot.getMap()));

        // LED commands
        // addCommand("LED Charge Up", new LedChargeUp(3000, Robot.getMap().leds)); DOESN'T WORK RN

        // Climber commands
        addCommand("Loosen Climber", new LoosenClimber());
        addCommand("Tighten Climber", new TightenClimber());
    }


    /**
     * <p> Adds any specific info that can only be accessed during teleop.
     * <p> THIS CURRENTLY DOES NOTHING
     */
    public static void teleopInit() {}


    /**
     * <p> Adds a number to the shuffleboard IF it doesn't already exist in the debug tab.
     * @param key The key (name) of the value being stored.
     * @param o The value that should be stored.
     */
    public static void addOnce(String key, Object o) {
        if (!SmartDashboard.containsKey(key)) {
            shuffleboard.debugTab.add(key, o);
        }
    }

    /**
     * <p> Adds a number to the shuffleboard, overwriting already existing values if called again.
     * @param key The key (name) of the the value being stored.
     * @param o The value that should be stored.
     */
    public static void add(String key, Object o) {
        shuffleboard.debugTab.add(key, o);
    }

    // Allows for the easy addition to any of the settings
    public void addShooterSetting(String key, Object o) { settings.add(shuffleboard.settingsShooter.addPersistent(key, o)); }
    public void addSwerveSetting(String key, Object o) { settings.add(shuffleboard.settingsSwerve.addPersistent(key, o)); }
    public void addIntakeSetting(String key, Object o) { settings.add(shuffleboard.settingsIntake.addPersistent(key, o)); }
    public void addClimberSetting(String key, Object o) { settings.add(shuffleboard.settingsClimber.addPersistent(key, o)); }
    public void addAutoSetting(String key, Object o) { settings.add(shuffleboard.settingsAuto.addPersistent(key, o)); }

    /**
     * Gets the value of a settings. Will default to 0.0 if the setting does not exist or is not set.
     * @param key The setting to be looking for
     * @return The value of the setting
     */
    public double getSettingNum(String key) {
        for (int i = 0; i < settings.size(); i++) {
            if (settings.get(i).getTitle().equals(key))
                return settings.get(i).getEntry().getDouble(0.0);
        }
        return 0.0;
    }


    /**
     * <p> Adds a command to the command layout in the debug tab that can be activated by pressing it.
     * @param command The command to be added.
     * @param key The name of the command.
     */
    public void addCommand(String key, Command command) {
        shuffleboard.debugCommands.add(key, command);
    }

}
