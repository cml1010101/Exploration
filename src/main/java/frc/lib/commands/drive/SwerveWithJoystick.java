package frc.lib.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.oi.OI;
import frc.lib.subsystems.drive.SwerveDrive;

public class SwerveWithJoystick extends CommandBase {
    private final SwerveDrive drive;
    private final double maxSpeed;
    private final boolean fieldRelative, useClosedLoop;
    private DoubleSupplier[] driveSuppliers;
    private Alliance alliance;
    public SwerveWithJoystick(SwerveDrive drive, OI oi, boolean fieldRelative, double maxSpeed)
    {
        addRequirements(drive);
        this.drive = drive;
        this.fieldRelative = fieldRelative;
        this.maxSpeed = maxSpeed;
        driveSuppliers = new DoubleSupplier[] {
            oi.getDriveSupplier(),
            oi.getDriveSupplier(),
            oi.getDriveSupplier()
        };
        this.useClosedLoop = oi.useClosedLoop();
    }
    @Override
    public void initialize()
    {
        alliance = DriverStation.getAlliance();
    }
    @Override
    public void execute()
    {
        if (fieldRelative)
        {
            Rotation2d headingOffset = alliance == Alliance.Red ? Rotation2d.fromDegrees(180) : new Rotation2d();
            drive.driveUsingChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        driveSuppliers[0].getAsDouble() * maxSpeed, 
                        driveSuppliers[2].getAsDouble() * maxSpeed, 
                        driveSuppliers[1].getAsDouble() * maxSpeed
                    ),
                    drive.getHeading().plus(headingOffset)
                ), useClosedLoop
            );
        }
        else
        {
            drive.driveUsingChassisSpeeds(
                new ChassisSpeeds(
                    driveSuppliers[0].getAsDouble() * maxSpeed, 
                    driveSuppliers[2].getAsDouble() * maxSpeed, 
                    driveSuppliers[1].getAsDouble() * maxSpeed
                ), useClosedLoop
            );
        }
    }
}
