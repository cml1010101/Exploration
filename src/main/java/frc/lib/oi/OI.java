package frc.lib.oi;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;

public abstract class OI {
    public abstract DoubleSupplier getDriveSupplier();
    public abstract boolean isFieldRelative();
    public abstract DoubleSupplier getManipulatorSupplier();
    public abstract GenericHID getDriverController();
    public abstract GenericHID getManipulatorController();
    public abstract boolean useClosedLoop();
}
