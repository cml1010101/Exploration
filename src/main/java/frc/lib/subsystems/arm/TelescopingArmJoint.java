package frc.lib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.arm.ArmJointExtend;
import frc.lib.commands.arm.ArmJointRetract;
import frc.lib.oi.OI;
import frc.lib.pneumatics.SmartSolenoid;
import frc.lib.pneumatics.SmartSolenoid.SmartSolenoidState;
import frc.lib.subsystems.SmartSubsystem;

public class TelescopingArmJoint extends SmartSubsystem implements ArmJoint {
    public static final class TelescopingArmJointConfiguration
    {
        private final double kRetractedLength, kExtendedLength, kExtensionTime, kRetractionTime;
        private final Rotation3d kRotation;
        private final Translation3d kFulcrumOffset;
        private final String name;
        public TelescopingArmJointConfiguration(double kRetractedLength, double kExtendedLength, double kExtensionTime,
            double kRetractionTime, Rotation3d kRotation, Translation3d kFulcrumOffset, String name)
        {
            this.kRetractedLength = kRetractedLength;
            this.kExtendedLength = kExtendedLength;
            this.kExtensionTime = kExtensionTime;
            this.kRetractionTime = kRetractionTime;
            this.kRotation = kRotation;
            this.kFulcrumOffset = kFulcrumOffset;
            this.name = name;
        }
    }
    public static enum TelescopingArmJointState implements ArmJointState
    {
        kExtended,
        kRetracted,
        kExtending,
        kRetracting
    }
    private final TelescopingArmJointConfiguration config;
    private final SmartSolenoid solenoid;
    private double estimatedLength;
    private final Timer timer;
    private TelescopingArmJointState state;
    private double waitTime;
    private final MechanismLigament2d mech;
    public TelescopingArmJoint(SmartSolenoid solenoid, TelescopingArmJointConfiguration config)
    {
        this.solenoid = solenoid;
        this.config = config;
        this.timer = new Timer();
        this.state = TelescopingArmJointState.kRetracted;
        this.estimatedLength = config.kRetractedLength;
        this.mech = new MechanismLigament2d(config.name, estimatedLength, 0);
    }
    public double getLength()
    {
        return estimatedLength;
    }
    public void extend()
    {
        if (state != TelescopingArmJointState.kExtended && state != TelescopingArmJointState.kExtending)
        {
            state = TelescopingArmJointState.kExtending;
            solenoid.setState(SmartSolenoidState.kOpen);
            if (state == TelescopingArmJointState.kRetracted)
            {
                timer.restart();
                waitTime = config.kExtensionTime;
            }
            else
            {
                double retractionPercent = timer.get() / config.kRetractionTime;
                double extensionPercent = 1 - retractionPercent;
                waitTime = config.kExtensionTime * extensionPercent;
            }
        }
    }
    public void retract()
    {
        if (state != TelescopingArmJointState.kRetracted && state != TelescopingArmJointState.kRetracting)
        {
            state = TelescopingArmJointState.kRetracting;
            if (solenoid.supportsReverse())
            {
                solenoid.setState(SmartSolenoidState.kReverse);
            }
            else
            {
                solenoid.setState(SmartSolenoidState.kClosed);
            }
            if (state == TelescopingArmJointState.kExtended)
            {
                timer.restart();
                waitTime = config.kRetractionTime;
            }
            else
            {
                double extensionPercent = timer.get() / config.kExtensionTime;
                double retractionPercent = 1 - extensionPercent;
                waitTime = config.kRetractionTime * retractionPercent;
            }
        }
    }
    @Override
    public Translation3d getEndPoint() {
        return new Translation3d(getLength(), getRotation())
            .plus(config.kFulcrumOffset);
    }
    @Override
    public Rotation3d getRotation() {
        return config.kRotation;
    }
    public Command getExtendCommand()
    {
        return new ArmJointExtend(this);
    }
    public Command getRetractCommand()
    {
        return new ArmJointRetract(this);
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return null;
    }
    @Override
    public void periodic()
    {
        switch (state)
        {
        case kExtending:
            if (timer.hasElapsed(waitTime))
            {
                timer.reset();
                state = TelescopingArmJointState.kExtended;
                estimatedLength = config.kExtendedLength;
            }
            else
            {
                estimatedLength += (config.kExtendedLength - config.kRetractedLength) * (0.02 / config.kExtensionTime);
            }
        case kRetracting:
            if (timer.hasElapsed(waitTime))
            {
                timer.reset();
                state = TelescopingArmJointState.kRetracted;
                estimatedLength = config.kRetractedLength;
            }
            else
            {
                estimatedLength -= (config.kExtendedLength - config.kRetractedLength) * (0.02 / config.kRetractionTime);
            }
        default:
            break;
        }
        mech.setLength(estimatedLength);
    }
    public TelescopingArmJointState getState() {
        return state;
    }
    @Override
    public void setState(ArmJointState state) {
        assert TelescopingArmJointState.class.isAssignableFrom(state.getClass());
        switch ((TelescopingArmJointState)state)
        {
        case kExtended:
            extend();
            break;
        case kRetracted:
            retract();
            break;
        default:
            break;
        }
    }
    @Override
    public boolean atState(ArmJointState state) {
        assert TelescopingArmJointState.class.isAssignableFrom(state.getClass());
        return (TelescopingArmJointState)state == getState();
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
        builder.addStringProperty("Current State", () -> getState().toString(), null);
        builder.addDoubleProperty("Length", () -> getLength(), null);
    }
    @Override
    public MechanismObject2d getMechanism()
    {
        return mech;
    }
    @Override
    public Translation3d getFulcrumOffset()
    {
        return config.kFulcrumOffset;
    }
}
