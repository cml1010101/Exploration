package frc.lib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.arm.ArmJointSetSpeed;
import frc.lib.commands.arm.ArmJointStopSpeed;
import frc.lib.motors.MotorGroup;
import frc.lib.oi.OI;
import frc.lib.subsystems.SmartSubsystem;

public class IntakeArmJoint extends SmartSubsystem implements ArmJoint {
    public static final class IntakeArmJointConfiguration
    {
        private final Translation3d kFulcrumOffset;
        private final Rotation3d kRotation;
        private final double closedLoopError;
        public IntakeArmJointConfiguration(Translation3d kFulcrumOffset, Rotation3d kRotation, double closedLoopError)
        {
            this.kFulcrumOffset = kFulcrumOffset;
            this.kRotation = kRotation;
            this.closedLoopError = closedLoopError;
        }
    }
    public static class IntakeArmJointState implements ArmJointState
    {
        private final double speed;
        private final boolean openLoop;
        public IntakeArmJointState(double speed, boolean openLoop)
        {
            this.speed = speed;
            this.openLoop = openLoop;
        }
    }
    private final MotorGroup group;
    private final IntakeArmJointConfiguration config;
    public IntakeArmJoint(MotorGroup group, IntakeArmJointConfiguration config)
    {
        this.group = group;
        this.config = config;
    }
    @Override
    public Translation3d getEndPoint() {
        return config.kFulcrumOffset;
    }
    @Override
    public Rotation3d getRotation() {
        return config.kRotation;
    }
    @Override
    public void setState(ArmJointState state) {
        assert IntakeArmJointState.class.isAssignableFrom(state.getClass());
        IntakeArmJointState intakeState = (IntakeArmJointState)state;
        if (intakeState.openLoop)
        {
            group.set(intakeState.speed);
        }
        else
        {
            group.setVelocity(intakeState.speed);
        }
    }
    @Override
    public boolean atState(ArmJointState state) {
        assert IntakeArmJointState.class.isAssignableFrom(state.getClass());
        IntakeArmJointState intakeState = (IntakeArmJointState)state;
        return intakeState.openLoop ? group.get() == intakeState.speed
            : Math.abs(group.getEncoderRPS() - intakeState.speed) <= config.closedLoopError;
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return new ArmJointStopSpeed(this);
    }
    public void setSpeed(double speed, boolean openLoop)
    {
        if (openLoop)
        {
            group.set(speed);
        }
        else
        {
            group.setVelocity(speed);
        }
    }
    public Command getSetSpeedCommand(double speed, boolean openLoop)
    {
        return new ArmJointSetSpeed(this, speed, openLoop);
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
        builder.addDoubleProperty("Speed (RPS)", group::getEncoderRPS, null);
    }
    @Override
    public MechanismObject2d getMechanism() {
        return new MechanismLigament2d("Intake", 0, 0);
    }
    @Override
    public Translation3d getFulcrumOffset()
    {
        return config.kFulcrumOffset;
    }
}
