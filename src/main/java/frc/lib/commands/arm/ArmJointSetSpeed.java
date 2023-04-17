package frc.lib.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.IntakeArmJoint;

public class ArmJointSetSpeed extends CommandBase {
    private final IntakeArmJoint arm;
    private final double speed;
    private final boolean openLoop;
    public ArmJointSetSpeed(IntakeArmJoint arm, double speed, boolean openLoop)
    {
        this.arm = arm;
        this.speed = speed;
        this.openLoop = openLoop;
        addRequirements(arm);
    }
    @Override
    public void initialize()
    {
        arm.setSpeed(speed, openLoop);
    }
}
