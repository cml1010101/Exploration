package frc.lib.gyros;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMU {
    public Rotation2d getHeading();
    public Rotation2d getPitchAngle();
    public Rotation2d getRollAngle();
    public Rotation2d getAngularVelocity();
    public void calibrate();
    public void reset();
    public boolean isCalibrating();
    public void setHeading(Rotation2d heading);
    public void setSimulationHeading(Rotation2d heading);
}
