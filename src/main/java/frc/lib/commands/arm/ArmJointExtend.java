package frc.lib.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.TelescopingArmJoint;
import frc.lib.subsystems.arm.TelescopingArmJoint.TelescopingArmJointState;

public class ArmJointExtend extends CommandBase {
    private final TelescopingArmJoint joint;
    public ArmJointExtend(TelescopingArmJoint joint)
    {
        this.joint = joint;
        addRequirements(joint);
    }
    @Override
    public void initialize()
    {
        joint.extend();
    }
    @Override
    public boolean isFinished()
    {
        return joint.getState() == TelescopingArmJointState.kExtended;
    }
}
