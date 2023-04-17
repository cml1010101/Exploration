package frc.lib.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.drive.TankDrive;

public class TankDriveMeters extends CommandBase {
    private final TankDrive drive;
    private final double targetDistance;
    private final double targetSpeed;
    private double startingLeft, startingRight;
    public TankDriveMeters(TankDrive drive, double speed, double meters)
    {
        addRequirements(drive);
        this.drive = drive;
        this.targetSpeed = speed;
        this.targetDistance = meters * Math.signum(speed);
    }
    @Override
    public void initialize()
    {
        startingLeft = drive.getLeftDistance();
        startingRight = drive.getRightDistance();
    }
    @Override
    public void execute()
    {
        drive.driveUsingChassisSpeeds(new ChassisSpeeds(
            targetSpeed, 0, 0
        ), true);
    }
    @Override
    public boolean isFinished()
    {
        return ((drive.getLeftDistance() - startingLeft) + (drive.getRightDistance() - startingRight)) / 2 > targetDistance;
    }
}
