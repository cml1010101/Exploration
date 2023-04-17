package frc.robot.oi;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.oi.OI;

public class DefaultOI extends OI {
    private final double deadband = 0.08;
    private final XboxController driverController = new XboxController(0);
    private final XboxController manipulatorController = new XboxController(1);
    private final ArrayList<DoubleSupplier> manipulatorSuppliers = new ArrayList<>(
        List.of(
            () -> -MathUtil.applyDeadband(manipulatorController.getLeftY(), deadband),
            () -> -MathUtil.applyDeadband(manipulatorController.getRightY(), deadband)
        )
    );
    private final ArrayList<DoubleSupplier> driverSuppliers = new ArrayList<>(
        List.of(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), deadband),
            () -> -MathUtil.applyDeadband(driverController.getRightX(), deadband),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), deadband)
        )
    );
    @Override
    public DoubleSupplier getManipulatorSupplier()
    {
        return manipulatorSuppliers.remove(0);
    }
    @Override
    public boolean isFieldRelative() {
        return true;
    }
    @Override
    public DoubleSupplier getDriveSupplier() {
        return driverSuppliers.remove(0);
    }
    @Override
    public GenericHID getDriverController() {
        return driverController;
    }
    @Override
    public GenericHID getManipulatorController() {
        return manipulatorController;
    }
    @Override
    public boolean useClosedLoop()
    {
        return true;
    }
}
