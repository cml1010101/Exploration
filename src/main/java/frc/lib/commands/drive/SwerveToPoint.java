package frc.lib.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.drive.SwerveDrive;

public class SwerveToPoint extends CommandBase {
    private final SwerveDrive drive;
    private final Pose2d pose;
    private final PathConstraints constraints;
    private Trajectory trajectory;
    private final Timer timer;
    public SwerveToPoint(SwerveDrive drive, Pose2d pose, PathConstraints constraints)
    {
        this.drive = drive;
        this.pose = pose;
        this.constraints = constraints;
        this.timer = new Timer();
        addRequirements(drive);
    }
    @Override
    public void initialize()
    {
        trajectory = PathPlanner.generatePath(constraints, new PathPoint(drive.getPose().getTranslation(),
            drive.getPose().getRotation()), new PathPoint(pose.getTranslation(), pose.getRotation()));
        timer.restart();
    }
    @Override
    public void execute()
    {
        ChassisSpeeds speeds = drive.getController().calculate(drive.getPose(), trajectory.sample(timer.get()), pose.getRotation());
        drive.driveUsingChassisSpeeds(speeds, true);
    }
    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds()) && drive.getController().atReference();
    }
}
