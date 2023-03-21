package frc.robot.subsystems.drive;

public class SwerveUtils {
    
    public static double angleError(double current, double target) {
        double error = target - current;
        if (error < -180) {
            error += 360;
        }
        if (error > 180) {
            error -= 360;
        }
        return error;
    }
}
