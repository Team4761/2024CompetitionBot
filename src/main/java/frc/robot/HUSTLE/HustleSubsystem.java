package frc.robot.HUSTLE;
import java.io.File;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HustleSubsystem extends SubsystemBase {
    
    private static final int MUSIC_MOTOR_PORT = Constants.SHOOTER_RIGHT_MOTOR_PORT;  // CAN
    private static final String THE_HOLY_FILE_NAME = "hustle.chrp";

    private static final String[] THE_HOLY_FILES = {
        "continental_way",
        "darude",
        "hustle",
        "spain_fixed2",
        "triforce",
        "zanzibar"
    };

    private TalonFX musicMotor;
    
    private TalonFX mm1;
    private TalonFX mm2;
    private TalonFX mm3;
    private TalonFX mm4;
    private TalonFX mm5;

    private Orchestra orchestra;



    /**
     * WARNING: This takes control of all talons on the robot.
     */
    public void playTheHustle() {
        musicMotor = new TalonFX(MUSIC_MOTOR_PORT);
        mm1 = new TalonFX(Constants.SHOOTER_LEFT_MOTOR_PORT);
        mm2 = new TalonFX(Constants.FL_DRIVE_PORT);
        mm3 = new TalonFX(Constants.FR_DRIVE_PORT);
        mm4 = new TalonFX(Constants.BL_DRIVE_PORT);
        mm5 = new TalonFX(Constants.BR_DRIVE_PORT);

        orchestra = new Orchestra();
        orchestra.addInstrument(musicMotor);
        orchestra.addInstrument(mm1);
        orchestra.addInstrument(mm2);
        orchestra.addInstrument(mm3);
        orchestra.addInstrument(mm4);
        orchestra.addInstrument(mm5);
        orchestra.loadMusic(THE_HOLY_FILE_NAME);
        orchestra.play();
    }

    /**
     * This both stops the music and frees up the motors for other use.
     */
    public void stopHustling() {
        orchestra.stop();
        orchestra.close();

        mm1.close();
        mm2.close();
        mm3.close();
        mm4.close();
        mm5.close();
    }


    public void play() {
        orchestra.play();
    }

    public void stop() {
        orchestra.stop();
    }
}
