package frc.lib.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.oi.OI;
import frc.lib.subsystems.arm.RotatingArmJoint;

public class ArmJointWithJoystick extends CommandBase {
    private final OI oi;
    private final RotatingArmJoint arm;
    private final DoubleSupplier manipulatorSupplier;
    public ArmJointWithJoystick(RotatingArmJoint arm, OI oi)
    {
        this.oi = oi;
        this.arm = arm;
        this.manipulatorSupplier = this.oi.getManipulatorSupplier();
        addRequirements(arm);
    }
    @Override
    public void execute()
    {
        arm.set(manipulatorSupplier.getAsDouble());
    }
}
