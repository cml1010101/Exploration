package frc.lib.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.drive.SwerveDrive;

public class SwerveStop extends CommandBase {
    private final SwerveDrive drive;
    public SwerveStop(SwerveDrive drive)
    {
        addRequirements(drive);
        this.drive = drive;
    }
    @Override
    public void initialize()
    {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };
        drive.setSwerveStates(states);
    }
    @Override
    public boolean isFinished()
    {
        return ((Math.abs(drive.getChassisSpeeds().vxMetersPerSecond) + Math.abs(drive.getChassisSpeeds().vyMetersPerSecond)) / 2) < 0.01;
    }
}
