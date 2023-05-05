package frc.lib.subsystems.arm;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.arm.ArmJointSetAngle;
import frc.lib.commands.arm.ArmJointWithJoystick;
import frc.lib.encoders.SmartEncoder;
import frc.lib.motors.MotorGroup;
import frc.lib.motors.MotorGroup.MotorEncoderMismatchException;
import frc.lib.oi.OI;
import frc.lib.subsystems.SmartSubsystem;

public class RotatingArmJoint extends SmartSubsystem implements ArmJoint {
    public static final class RotatingArmJointConfiguration
    {
        private final double kMotorGearRatio, kEncoderGearRatio, kWeight, kInitialLength, kEncoderSyncMaxSpeed;
        private final Rotation2d kMinAngle, kMaxAngle, kTolerance, kOffset;
        private final boolean kSupportsSmartPosition, linkExternalEncoderToInternalEncoder, 
            syncExternalEncoderToIntegradeEncoderRegularly, syncExternalEncoderToIntegradeEncoderOnStartup,
            enableContinuousInput;
        private final Translation3d kFulcrumOffset;
        private final Rotation3d kRotation;
        public RotatingArmJointConfiguration(double kMotorGearRatio, double kEncoderGearRatio, double kFeedforward, double kWeight,
            double kInitialLength, Rotation2d kMinAngle, Rotation2d kMaxAngle, Rotation2d kTolerance, Rotation2d kOffset,
            boolean kSupportsSmartPosition, boolean linkExternalEncoderToInternalEncoder, 
            boolean syncExternalEncoderToIntegradeEncoderRegularly, boolean syncExternalEncoderToIntegradeEncoderOnStartup,
            double kEncoderSyncMaxSpeed, Translation3d kFulcrumOffset, Rotation3d kRotation, boolean enableContinuousInput)
        {
            this.kMotorGearRatio = kMotorGearRatio;
            this.kEncoderGearRatio = kEncoderGearRatio;
            this.kWeight = kWeight;
            this.kInitialLength = kInitialLength;
            this.kMinAngle = kMinAngle;
            this.kMaxAngle = kMaxAngle;
            this.kTolerance = kTolerance;
            this.kOffset = kOffset;
            this.kSupportsSmartPosition = kSupportsSmartPosition;
            this.linkExternalEncoderToInternalEncoder = linkExternalEncoderToInternalEncoder;
            this.syncExternalEncoderToIntegradeEncoderOnStartup = syncExternalEncoderToIntegradeEncoderOnStartup;
            this.syncExternalEncoderToIntegradeEncoderRegularly = syncExternalEncoderToIntegradeEncoderRegularly;
            this.kEncoderSyncMaxSpeed = kEncoderSyncMaxSpeed;
            this.kFulcrumOffset = kFulcrumOffset;
            this.kRotation = kRotation;
            this.enableContinuousInput = enableContinuousInput;
        }
    }
    public static final class RotatingArmJointState implements ArmJointState
    {
        private final Rotation2d targetAngle;
        public RotatingArmJointState(Rotation2d targetAngle)
        {
            this.targetAngle = targetAngle;
        }
        @Override
        public boolean equals(Object other)
        {
            if (other.getClass() != RotatingArmJointState.class) return false;
            return ((RotatingArmJointState)other).targetAngle.equals(targetAngle);
        }
    }
    private final MotorGroup motorGroup;
    private final SmartEncoder encoder;
    private final SmartSingleJointedArmSim armSimulator;
    private final RotatingArmJointConfiguration config;
    private final Function<Rotation2d, Double> feedforward;
    private final DoubleSupplier moi;
    private double lastRecordedMOI;
    private final LinearSystem<N2, N1, N1> simulationID;
    private final MechanismLigament2d mech;
    public RotatingArmJoint(ShuffleboardTab tab, String name, MotorGroup motorGroup, SmartEncoder encoder, Function<Rotation2d, Double> feedforward,
        DoubleSupplier moi, RotatingArmJointConfiguration config)
    {
        super(tab, name);
        this.motorGroup = motorGroup;
        this.encoder = encoder;
        this.config = config;
        this.feedforward = feedforward;
        this.moi = moi;
        if (config.linkExternalEncoderToInternalEncoder)
        {
            try {
                motorGroup.linkEncoder(encoder);
            } catch (MotorEncoderMismatchException e) {
                e.printStackTrace();
            }
            if (config.enableContinuousInput)
            {
                motorGroup.enableContinuousInput(true);
            }
        }
        else if (config.syncExternalEncoderToIntegradeEncoderOnStartup)
        {
            motorGroup.setEncoderRotations(encoder.getEncoderRotations() / config.kEncoderGearRatio * config.kMotorGearRatio);
        }
        if (RobotBase.isReal())
        {
            armSimulator = null;
            simulationID = null;
        }
        else
        {
            if (moi == null)
            {
                simulationID = LinearSystemId.createSingleJointedArmSystem(motorGroup.getGearbox(),
                    SingleJointedArmSim.estimateMOI(config.kInitialLength, config.kWeight), config.kMotorGearRatio);
            }
            else
            {
                simulationID = LinearSystemId.createSingleJointedArmSystem(motorGroup.getGearbox(),
                    lastRecordedMOI = moi.getAsDouble(), config.kMotorGearRatio);
            }
            armSimulator = new SmartSingleJointedArmSim(simulationID, motorGroup.getGearbox(), config.kMotorGearRatio,  
                config.kInitialLength, config.kMinAngle.getRadians(), config.kMaxAngle.getRadians(), false);
        }
        if (config.kMinAngle != null)
        {
            motorGroup.setReverseSoftLimit(config.kMinAngle.minus(config.kOffset).getRotations() * (config.linkExternalEncoderToInternalEncoder ?
                config.kEncoderGearRatio : config.kMotorGearRatio));
        }
        else
        {
            motorGroup.disableReverseSoftLimit();
        }
        if (config.kMaxAngle != null)
        {
            motorGroup.setForwardSoftLimit(config.kMaxAngle.minus(config.kOffset).getRotations() * (config.linkExternalEncoderToInternalEncoder ?
                config.kEncoderGearRatio : config.kMotorGearRatio));
        }
        else
        {
            motorGroup.disableForwardSoftLimit();
        }
        mech = new MechanismLigament2d(name, 0, getAngle().getDegrees());
    }
    public RotatingArmJoint(String name, MotorGroup motorGroup, SmartEncoder encoder, Function<Rotation2d, Double> feedforward,
        DoubleSupplier moi, RotatingArmJointConfiguration config)
    {
        this(null, name, motorGroup, encoder, feedforward, moi, config);
    }
    public RotatingArmJoint(String name, MotorGroup motorGroup, SmartEncoder encoder, DoubleSupplier moi, RotatingArmJointConfiguration config)
    {
        this(name, motorGroup, encoder, null, moi, config);
    }
    public RotatingArmJoint(String name, MotorGroup motorGroup, SmartEncoder encoder, Function<Rotation2d, Double> feedforward,
        RotatingArmJointConfiguration config)
    {
        this(name, motorGroup, encoder, feedforward, null, config);
    }
    public RotatingArmJoint(String name, MotorGroup motorGroup, SmartEncoder encoder, RotatingArmJointConfiguration config)
    {
        this(name, motorGroup, encoder, null, null, config);
    }
    public Rotation2d getAngle()
    {
        return Rotation2d.fromRotations(encoder.getEncoderRotations() / config.kEncoderGearRatio).plus(config.kOffset);
    }
    public void setTargetAngle(Rotation2d targetAngle)
    {
        if (config.kSupportsSmartPosition)
        {
            motorGroup.setSmartPosition(targetAngle.minus(config.kOffset).getRotations());
        }
        else
        {
            motorGroup.setPosition(targetAngle.minus(config.kOffset).getRotations());
        }
    }
    public void set(double percent)
    {
        if (feedforward != null)
        {
            motorGroup.set(percent + feedforward.apply(getAngle()));
        }
    }
    public void resetPosition(Rotation2d newPosition)
    {
        encoder.resetEncoderRotations(newPosition.getRotations() * config.kEncoderGearRatio);
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return new ArmJointWithJoystick(this, oi);
    }
    public Command getGoToAngleCommand(Rotation2d angle)
    {
        return new ArmJointSetAngle(this, angle, config.kTolerance);
    }
    @Override
    public void periodic()
    {
        if (!config.linkExternalEncoderToInternalEncoder && config.syncExternalEncoderToIntegradeEncoderRegularly)
        {
            if (Math.abs(encoder.getEncoderRPS() / config.kEncoderGearRatio) < config.kEncoderSyncMaxSpeed)
            {
                motorGroup.setEncoderRotations(encoder.getEncoderRotations() / config.kEncoderGearRatio * config.kMotorGearRatio);
            }
        }
        if (moi != null && moi.getAsDouble() != lastRecordedMOI && RobotBase.isSimulation())
        {
            lastRecordedMOI = moi.getAsDouble();
            var newSystem = LinearSystemId.createSingleJointedArmSystem(motorGroup.getGearbox(),
                lastRecordedMOI, config.kMotorGearRatio);
            armSimulator.loadNewPlant(newSystem);
        }
    }
    @Override
    public void simulationPeriodic()
    {
        armSimulator.setInput(motorGroup.getSimulationOutputVoltage());
        armSimulator.update(0.02);
        encoder.setSimulationRotations(Rotation2d.fromRadians(armSimulator.getAngleRads()).getRotations());
        encoder.setSimulationRPS(Rotation2d.fromRadians(armSimulator.getVelocityRadPerSec()).getRotations());
    }
    @Override
    public Translation3d getEndPoint()
    {
        return config.kFulcrumOffset;
    }
    @Override
    public Rotation3d getRotation()
    {
        return config.kRotation
            .plus(new Rotation3d(0, getAngle().getRadians(), 0));
    }
    @Override
    public void setState(ArmJointState state) {
        assert RotatingArmJointState.class.isAssignableFrom(state.getClass());
        setTargetAngle(((RotatingArmJointState)state).targetAngle);
    }
    @Override
    public boolean atState(ArmJointState state) {
        assert RotatingArmJointState.class.isAssignableFrom(state.getClass());
        return Math.abs(((RotatingArmJointState)state).targetAngle.getDegrees() - getAngle().getDegrees())
            <= config.kTolerance.getDegrees();
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
        builder.addDoubleProperty("Angle", () -> getAngle().getDegrees(), null);
    }
    @Override
    public MechanismObject2d getMechanism() {
        return mech;
    }
    @Override
    public Translation3d getFulcrumOffset()
    {
        return config.kFulcrumOffset;
    }
}
