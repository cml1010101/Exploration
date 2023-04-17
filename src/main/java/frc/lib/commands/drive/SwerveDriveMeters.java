package frc.lib.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.drive.SwerveDrive;

public class SwerveDriveMeters extends CommandBase {
    private final SwerveDrive drive;
    private final double targetDistance;
    private final double targetSpeed;
    private double startingLeft, startingRight;
    public SwerveDriveMeters(SwerveDrive drive, double speed, double meters)
    {
        addRequirements(drive);
        this.drive = drive;
        this.targetSpeed = speed;
        this.targetDistance = meters * Math.signum(speed);
    }
    @Override
    public void initialize()
    {
        startingLeft = drive.getPositions()[0].distanceMeters;
        startingRight = drive.getPositions()[1].distanceMeters;
    }
    @Override
    public void execute()
    {
        drive.driveUsingChassisSpeeds(new ChassisSpeeds(targetSpeed, 0, 0), true);
    }
    @Override
    public boolean isFinished()
    {
        return (Math.abs(drive.getPositions()[0].distanceMeters - startingLeft) + Math.abs(drive.getPositions()[1].distanceMeters - startingRight)) / 2 > targetDistance;
    }
}
