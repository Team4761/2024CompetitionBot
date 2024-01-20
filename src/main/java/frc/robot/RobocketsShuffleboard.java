package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobocketsShuffleboard {
    public static void teleopInit() {
        addNumber("Shooter In Speed", 0.5);
        addNumber("Shooter Out Speed", 0.5);
        addNumber("Shooter Intake Speed", 0.5);
        addNumber("Shooter Outtake Speed", 0.5);
    }


    public static void addNumber(String key, double num) {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, num);
        }
    }
}
