package frc.lib.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.RotatingArmJoint;

public class ArmJointSetAngle extends CommandBase {
    private final RotatingArmJoint arm;
    private final Rotation2d targetAngle, tolerance;
    public ArmJointSetAngle(RotatingArmJoint arm, Rotation2d targetAngle, Rotation2d tolerance)
    {
        this.arm = arm;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        addRequirements(arm);
    }
    @Override
    public void initialize()
    {
        arm.setTargetAngle(targetAngle);
    }
    @Override
    public boolean isFinished()
    {
        return Math.abs(arm.getAngle().getDegrees() - targetAngle.getDegrees()) <= tolerance.getDegrees();
    }
}
