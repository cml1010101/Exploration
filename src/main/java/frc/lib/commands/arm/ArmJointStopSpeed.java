package frc.lib.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.subsystems.arm.IntakeArmJoint;

public class ArmJointStopSpeed extends CommandBase {
    private final IntakeArmJoint intake;
    public ArmJointStopSpeed(IntakeArmJoint intake)
    {
        this.intake = intake;
    }
    @Override
    public void initialize()
    {
        intake.setSpeed(0, true);
    }
}
