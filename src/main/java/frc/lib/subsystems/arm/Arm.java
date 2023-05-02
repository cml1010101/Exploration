package frc.lib.subsystems.arm;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.arm.ArmPositionArm;
import frc.lib.subsystems.arm.ArmJoint.ArmJointState;

public class Arm {
    public static final class ArmState
    {
        private final List<ArmJointState> states;
        public ArmState(ArmJointState... states)
        {
            this.states = List.of(states);
        }
        @Override
        public boolean equals(Object other)
        {
            if (other.getClass() != ArmState.class) return false;
            ArmState state = (ArmState)other;
            if (state.states.size() != states.size()) return false;
            for (int i = 0; i < states.size(); i++)
            {
                if (!states.get(i).equals(state.states.get(i))) return false;
            }
            return true;
        }
    }
    private final List<ArmJoint> joints;
    public Arm(ShuffleboardTab tab, ArmJoint... joints)
    {
        this.joints = List.of(joints);
        if (tab != null)
        {
            for (int i = 0; i < joints.length; i++)
            {
                tab.add("Joint #" + (i + 1), joints[i]);
            }
            tab.add("Mechanism View", getMechanism());
        }
    }
    public Arm(ArmJoint... joints)
    {
        this(null, joints);
    }
    public void setState(ArmState state)
    {
        for (int i = 0; i < joints.size(); i++)
        {
            joints.get(i).setState(state.states.get(i));
        }
    }
    public boolean atState(ArmState state)
    {
        for (int i = 0; i < joints.size(); i++)
        {
            if (!joints.get(i).atState(state.states.get(i)))
            {
                return false;
            }
        }
        return true;
    }
    public ArmJoint[] getAllJoints()
    {
        return (ArmJoint[])joints.toArray();
    }
    public Command getPositionArmCommand(ArmState state)
    {
        return new ArmPositionArm(this, state);
    }
    public Translation3d getEndPoint()
    {
        Translation3d translation = new Translation3d();
        Rotation3d rotation = new Rotation3d();
        for (ArmJoint joint : joints)
        {
            translation = translation.plus(joint.getEndPoint().rotateBy(rotation));
            rotation = rotation.plus(joint.getRotation());
        }
        return translation;
    }
    public Rotation3d getOrientation()
    {
        Rotation3d rotation = new Rotation3d();
        for (ArmJoint joint : joints)
        {
            rotation = rotation.plus(joint.getRotation());
        }
        return rotation;
    }
    public Mechanism2d getMechanism()
    {
        Mechanism2d mech = new Mechanism2d(10, 10);
        if (joints.size() > 0)
        {
            var root = mech.getRoot("Arm", joints.get(0).getFulcrumOffset().getX(), joints.get(0).getFulcrumOffset().getY());
            var currentMech = root.append(joints.get(0).getMechanism());
            for (int i = 1; i < joints.size(); i++)
            {
                currentMech = currentMech.append(
                    new MechanismLigament2d("Joint " + (i + 1) + Math.random() + " Fulcrum", joints.get(i).getFulcrumOffset().getNorm(),
                    Math.toDegrees(Math.atan(joints.get(i).getFulcrumOffset().getZ() / joints.get(i).getFulcrumOffset().getY())))
                );
                currentMech = currentMech.append(joints.get(i).getMechanism());
            }
        }
        return mech;
    }
}
