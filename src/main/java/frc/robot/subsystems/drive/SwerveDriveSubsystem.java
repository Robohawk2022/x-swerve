package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveModuleState2;

public class SwerveDriveSubsystem extends SubsystemBase {

    public static final ChassisSpeeds STOP = new ChassisSpeeds(0, 0, 0);

    private final SwerveDrive drive;

    public SwerveDriveSubsystem() {
        File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            this.drive  = new SwerveParser(configDir).createSwerveDrive();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public double getHeadingDegrees() {
        return drive.getYaw().getDegrees();
    }

    public void zeroGyro() {
        drive.zeroGyro();
    }

    public void setSwerveModuleStates(SwerveModuleState [] states) {
        SwerveModuleState2 [] s = new SwerveModuleState2[4];
        for (int i=0; i<states.length; i++) {
            s[i] = new SwerveModuleState2(states[i].speedMetersPerSecond, states[i].angle, 0);
        }
        drive.setModuleStates(s, true);
    }

    public void stop() {
        driveRobotRelative(STOP);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelative) {        
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, drive.getYaw());
        driveRobotRelative(robotRelative);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelative) {
        drive.setChassisSpeeds(robotRelative);
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
    }

    public Command lockWheelsCommand() {
        return new InstantCommand(drive::lockPose, this);
    }

    public Command zeroGyroCommand() {
        return new InstantCommand(drive::zeroGyro, this);
    }

    public Command swerveTeleopCommand(Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier fieldRelativeSupplier) {
        return new SwerveTeleopCommand(this, speedSupplier, fieldRelativeSupplier);
    }

    public Command alignCommand(double target) {
        return new SwerveAlignCommand(this, target);
    }

    public Command pathCommand(String path, double maxSpeed, Map<String,Command> events) {
        PathConstraints constraints = new PathConstraints(maxSpeed, maxSpeed);
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                drive::getPose,
                drive::resetOdometry,
                new SwerveDriveKinematics(drive.swerveDriveConfiguration.moduleLocationsMeters),
                new PIDConstants(5, 0, 0),
                new PIDConstants(5, 0, 0),
                this::setSwerveModuleStates,
                events,
                this);
        return autoBuilder.fullAuto(pathGroup);
    }
}
