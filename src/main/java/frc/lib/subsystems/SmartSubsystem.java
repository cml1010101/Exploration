package frc.lib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.oi.OI;

public abstract class SmartSubsystem extends SubsystemBase {
    public abstract Command getDefaultCommand(OI oi);
}
