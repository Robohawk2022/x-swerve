package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlignCommand extends CommandBase {
    
    private final SwerveDriveSubsystem drive;
    private final double target;
    private double current;
    private double error;
    private double output;
    private boolean done;

    public SwerveAlignCommand(SwerveDriveSubsystem drive, double target) {

        this.drive = drive;
        this.target = target;

        addRequirements(drive);

        SmartDashboard.putData("SwerveAlignCommand-"+target, builder -> {
            builder.addBooleanProperty("Done", () -> done, null);
            builder.addDoubleProperty("Current", () -> current, null);
            builder.addDoubleProperty("Error", () -> error, null);
            builder.addDoubleProperty("Output", () -> output, null);
        });
    }
    
    @Override
    public void initialize() {
        done = false;
        current = 0;
        error = 0;
        output = 0;
    }

    @Override
    public void execute() {

        current = drive.getHeadingDegrees();
        error = SwerveUtils.angleError(current, target);
        output = SwerveConfig.headingController.calculate(error);

        if (output != 0.0) {
            drive.driveRobotRelative(new ChassisSpeeds(0, 0, output));
        } else {
            drive.stop();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
