package frc.lib.gyros;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class NavX extends AHRS implements IMU {
    private final int simulationHandle;
    private final SimDouble headingSimulation;
    public NavX()
    {
        super();
        if (RobotBase.isSimulation())
        {
            simulationHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            headingSimulation = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simulationHandle, "Yaw"));
        }
        else
        {
            simulationHandle = -1;
            headingSimulation = null;
        }
    }
    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getAngle());
    }
    @Override
    public void setHeading(Rotation2d heading) {
        setAngleAdjustment(heading.minus(getHeading()).plus(Rotation2d.fromDegrees(getAngleAdjustment())).getDegrees());
    }
    @Override
    public Rotation2d getPitchAngle() {
        return Rotation2d.fromDegrees(getPitch());
    }
    @Override
    public Rotation2d getRollAngle() {
        return Rotation2d.fromDegrees(getRoll());
    }
    @Override
    public double getAngle()
    {
        return -super.getAngle();
    }
    @Override
    public void setSimulationHeading(Rotation2d heading)
    {
        headingSimulation.set(-MathUtil.inputModulus(heading.getDegrees(), -180, 180) - getAngleAdjustment());
    }
    @Override
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromDegrees(super.getRate());
    }
}
