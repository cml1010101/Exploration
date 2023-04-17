package frc.lib.commands.drive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.drive.TankDrive;

public class TankStop extends CommandBase {
    private final TankDrive drive;
    public TankStop(TankDrive drive)
    {
        addRequirements(drive);
        this.drive = drive;
    }
    @Override
    public void initialize()
    {
        drive.driveUsingSpeeds(new DifferentialDriveWheelSpeeds(0, 0));
    }
    @Override
    public boolean isFinished()
    {
        return ((Math.abs(drive.getSpeeds().leftMetersPerSecond) + Math.abs(drive.getSpeeds().rightMetersPerSecond)) / 2) < 0.01;
    }
}
