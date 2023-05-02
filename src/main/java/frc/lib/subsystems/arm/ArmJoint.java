package frc.lib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmJoint extends Subsystem, Sendable {
    public static interface ArmJointState
    {
    }
    public static interface ArmJointSolution
    {
    }
    public static final class ArmJointRange
    {
        protected final Rotation3d minAngle, maxAngle;
        protected final double minRadius, maxRadius;
        public ArmJointRange(Rotation3d minAngle, Rotation3d maxAngle, double minRadius, double maxRadius)
        {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
            this.minRadius = minRadius;
            this.maxRadius = maxRadius;
        }
    }
    public String getName();
    public Translation3d getEndPoint();
    public Rotation3d getRotation();
    public void setState(ArmJointState state);
    public boolean atState(ArmJointState state);
    public MechanismObject2d getMechanism();
    public Translation3d getFulcrumOffset();
}
