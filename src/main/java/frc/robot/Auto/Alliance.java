package frc.robot.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Alliance extends SendableChooser<String> {
    private final String kBlueAlliance = "blueAlliance";
    private final String kRedAlliance = "redAlliance";

    public Alliance() {
        setDefaultOption("Blue Alliance", kBlueAlliance);
        addOption("Red Alliance", kRedAlliance);
    }
}
