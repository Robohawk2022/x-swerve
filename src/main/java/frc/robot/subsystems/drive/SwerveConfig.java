package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import frc.robot.util.SpeedController;

public class SwerveConfig {

    public static final SpeedController headingController = new SpeedController(
        2,   // minimum allowable heading drift
        30,  // heading drift at which max correction speed is applied
        Units.degreesToRadians(10),    // minimum correction speed
        Units.degreesToRadians(120));  // maximum correction speed

}
