package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveXYCommand extends CommandBase {

    private final SwerveDriveSubsystem drive;
    private final double vx;
    private final double vy;
    private final double duration;

    public SwerveDriveXYCommand(SwerveDriveSubsystem drive, double vx, double vy, double duration) {
        this.drive = drive;
        this.vx = vx;
        this.vy = vy;
        this.duration = duration;
    }

    private double speedAt(double time) {
        
    }




}
