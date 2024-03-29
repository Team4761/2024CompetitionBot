package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathStuff {
    // % except negative numbers return positive ex. -13%63 = 51 instead of -13
    public static double modulus(double a, double b) {
        double c = a%b;
        if(c<0) {
            c+=b;
        }
        return c;
    }
    //angle subtraction but will result in -180 to 180 degrees
    public static Rotation2d subtract(Rotation2d a, Rotation2d b) {
        return new Rotation2d(
        modulus(modulus(a.getRadians(), Math.PI*2) - modulus(b.getRadians(), Math.PI*2) + Math.PI , Math.PI*2) - Math.PI);
        //(a%360 - b%360 + 180 ) % 360 - 180;
        // initial mod 360 not needed 
    }

    public static Rotation2d multiply(Rotation2d a, double b) {
        return new Rotation2d(
            a.getRadians()*b);
        //modulus(a.getRadians()*b+Math.PI, Math.PI*2) - Math.PI);
        //(a%360 - b%360 + 180 ) % 360 - 180;
        // initial mod 360 not needed 
    }

    public static Rotation2d negative(Rotation2d a) {
        return new Rotation2d(-a.getRadians());
    }

    public static ChassisSpeeds switchSpeedAndStrafe(ChassisSpeeds input) {
        return new ChassisSpeeds(input.vyMetersPerSecond, input.vxMetersPerSecond, input.omegaRadiansPerSecond);
    }

    public static ChassisSpeeds invert(ChassisSpeeds input) {
        return new ChassisSpeeds(input.vxMetersPerSecond, -input.vyMetersPerSecond, input.omegaRadiansPerSecond);
    }
}
