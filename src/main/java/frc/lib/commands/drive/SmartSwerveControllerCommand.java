package frc.lib.commands.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SmartSwerveControllerCommand extends SwerveControllerCommand {
    private final HolonomicDriveController controller;
    public SmartSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, SwerveDriveKinematics kinematics,
        HolonomicDriveController controller, Consumer<SwerveModuleState[]> outputModuleStates,
        Subsystem... requirements)
    {
        super(trajectory, pose, kinematics, controller, outputModuleStates, requirements);
        this.controller = controller;
    }
    @Override
    public boolean isFinished()
    {
        return super.isFinished() && controller.atReference();
    }
}
