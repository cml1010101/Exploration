package frc.lib.commands.drive;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SmartRamseteControllerCommand extends RamseteCommand {
    private final RamseteController controller;
    public SmartRamseteControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose, DifferentialDriveKinematics kinematics,
        RamseteController controller, BiConsumer<Double, Double> outputModuleStates,
        Subsystem... requirements)
    {
        super(trajectory, pose, controller, kinematics, outputModuleStates, requirements);
        this.controller = controller;
    }
    @Override
    public boolean isFinished()
    {
        return super.isFinished() && controller.atReference();
    }
}
