package frc.lib.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.TelescopingArmJoint;
import frc.lib.subsystems.arm.TelescopingArmJoint.TelescopingArmJointState;

public class ArmJointRetract extends CommandBase {
    private final TelescopingArmJoint joint;
    public ArmJointRetract(TelescopingArmJoint joint)
    {
        this.joint = joint;
        addRequirements(joint);
    }
    @Override
    public void initialize()
    {
        joint.retract();
    }
    @Override
    public boolean isFinished()
    {
        return joint.getState() == TelescopingArmJointState.kRetracted;
    }
}
