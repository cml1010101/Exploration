package frc.lib.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.Arm;
import frc.lib.subsystems.arm.Arm.ArmState;

public class ArmPositionArm extends CommandBase {
    private final ArmState state;
    private final Arm arm;
    public ArmPositionArm(Arm arm, ArmState state)
    {
        this.arm = arm;
        this.state = state;
        addRequirements(arm.getAllJoints());
    }
    @Override
    public void initialize()
    {
        arm.setState(state);
    }
    @Override
    public boolean isFinished()
    {
        return arm.atState(state);
    }
}
