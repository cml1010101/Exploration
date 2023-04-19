package frc.lib.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.oi.OI;
import frc.lib.subsystems.drive.TankDrive;

public class TankWithJoystick extends CommandBase {
    private final OI oi;
    private final TankDrive drive;
    private DoubleSupplier[] driveSuppliers;
    public TankWithJoystick(TankDrive drive, OI oi)
    {
        addRequirements(drive);
        this.oi = oi;
        this.drive = drive;
    }
    @Override
    public void initialize()
    {
        driveSuppliers = new DoubleSupplier[] {
            oi.getDriveSupplier(),
            oi.getDriveSupplier()
        };
    }
    @Override
    public void execute()
    {
        if (oi.useClosedLoop())
        {
            drive.driveUsingChassisSpeeds(new ChassisSpeeds(
                driveSuppliers[0].getAsDouble() * drive.getMaxSpeed(),
                0,
                driveSuppliers[1].getAsDouble() * drive.getMaxSpeed()
            ), true);
        }
        else
        {
            drive.driveUsingChassisSpeeds(new ChassisSpeeds(
                driveSuppliers[0].getAsDouble(),
                0,
                driveSuppliers[1].getAsDouble()
            ), false);
        }
    }
}
