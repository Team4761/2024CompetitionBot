package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class Constants
{
    public static int CONTROLLER_PORT = 0;   // On Dashboard under controller section

    // Swerve
    public static int FL_DRIVE_PORT = 1;    // CAN Port.
    public static int FR_DRIVE_PORT = 8;    // CAN Port.
    public static int BL_DRIVE_PORT = 2;    // CAN Port.
    public static int BR_DRIVE_PORT = 6;    // CAN Port.

    public static int FL_ROTATE_PORT = 4;   // CAN Port.
    public static int FR_ROTATE_PORT = 5;   // CAN Port.
    public static int BL_ROTATE_PORT = 3;   // CAN Port.
    public static int BR_ROTATE_PORT = 7;   // CAN Port.

    public static int FL_ENCODER_PORT = 1;  // DIO Port?
    public static int FR_ENCODER_PORT = 4;  // DIO Port?
    public static int BL_ENCODER_PORT = 3;  // DIO Port?
    public static int BR_ENCODER_PORT = 2;  // DIO Port?

    // Intake
    public static int INTAKE_RIGHT_ROTATE_PORT = 5; // CAN Port.
    public static int INTAKE_LEFT_ROTATE_PORT = 4;  // CAN Port.
    public static int INTAKE_RIGHT_PORT = 1;        // CAN Port.
    public static int INTAKE_LEFT_PORT = 3;         // CAN Port

    //Vision
    //TODO: Replace with real values ASAP
    public static Transform3d robotToCamera = new Transform3d(.5, 0, .5, new Rotation3d(0, 0, 0));
}