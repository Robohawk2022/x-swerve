package frc.robot.subsystems.drive;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTeleopCommand extends CommandBase {

    public static final int MAX_WAIT = 50;

    private final SwerveDriveSubsystem drive;
    private final Supplier<ChassisSpeeds> speedSupplier;
    private final BooleanSupplier fieldRelativeSupplier;
    private int done;

    public SwerveTeleopCommand(SwerveDriveSubsystem drive, Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier fieldRelativeSupplier) {
        this.drive = drive;
        this.speedSupplier = speedSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        addRequirements(drive);
    }

    public void initialize() {
        done = MAX_WAIT;
    }

    public void execute() {
        ChassisSpeeds speeds = speedSupplier.get();
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            done -= 1;
        }
        if (fieldRelativeSupplier.getAsBoolean()) {
            drive.driveFieldRelative(speeds);
        } else {
            drive.driveRobotRelative(speeds);
        }
    }

    public boolean isFinished() {
        return done == 0;
    }
}
