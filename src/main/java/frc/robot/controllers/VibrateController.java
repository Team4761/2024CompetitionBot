package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class VibrateController extends Command {
    private XboxController controller;
    private RumbleType rmblType;
    private double intensity;
    private long endTime;

    // controller, rumbletype left/right/both, intensity 0-1, length in seconds
    public VibrateController (XboxController control, double intensity) {
        controller = control;
        rmblType = RumbleType.kBothRumble;
        this.intensity = intensity;
        endTime = System.currentTimeMillis()+400;
    }
    public VibrateController (XboxController control, double intensity, double length) {
        controller = control;
        rmblType = RumbleType.kBothRumble;
        this.intensity = intensity;
        endTime = System.currentTimeMillis()+(long)(length*1000);
    }
    public VibrateController (XboxController control, RumbleType rumbleType, double intensity) {
        controller = control;
        rmblType = rumbleType;
        this.intensity = intensity;
        endTime = System.currentTimeMillis()+400;
    }
    public VibrateController (XboxController control, RumbleType rumbleType, double intensity, double length) {
        controller = control;
        rmblType = rumbleType;
        this.intensity = intensity;
        endTime = System.currentTimeMillis()+(long)(length*1000);
    }

    @Override
    public void execute() {
        controller.setRumble(rmblType, intensity);
    }

    @Override
    public boolean isFinished() {
        if (endTime <= System.currentTimeMillis())
            return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(rmblType, 0);
    }
}
