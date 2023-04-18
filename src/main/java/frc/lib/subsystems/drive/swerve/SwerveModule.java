package frc.lib.subsystems.drive.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.encoders.SmartAbsoluteEncoder;
import frc.lib.motors.MotorGroup;

public class SwerveModule implements Sendable {
    public static final class SwerveModuleConfiguration
    {
        private final double kDriveGearRatio, kTurnGearRatio, kDriveDistancePerRevolution, kEncoderSyncMaxSpeed;
        private final boolean kUseSmartControl, kSyncIntegratedWithAbsoluteRegularly, kSyncIntegratedWithAbsoluteOnStartup;
        /**
         * Creates a new SwerveModuleConfiguration.
         * @param driveGearRatio the drive gear ratio
         * @param turnGearRatio the turn gear ratio
         * @param wheelDiameter the wheel diameter
         * @param encoderSyncMaxSpeed the maximum speed that the turn motor can be going in order to sync the integrated sensor
         * in the motor with the absolute encoder.
         * @param useSmartControl whether or not to use smart control. (MotionMagic for )
         * @param syncIntegratedWithAbsoluteRegularly whether to sync regularly when motion is less than encoderSyncMaxSpeed.
         * @param syncIntegratedWithAbsoluteOnStartup whether to sync on startup.
         */
        public SwerveModuleConfiguration(double driveGearRatio, double turnGearRatio, double wheelDiameter,
            double encoderSyncMaxSpeed, boolean useSmartControl, boolean syncIntegratedWithAbsoluteRegularly,
            boolean syncIntegratedWithAbsoluteOnStartup)
        {
            this.kDriveGearRatio = driveGearRatio;
            this.kTurnGearRatio = turnGearRatio;
            this.kDriveDistancePerRevolution = wheelDiameter * Math.PI / driveGearRatio;
            this.kEncoderSyncMaxSpeed = encoderSyncMaxSpeed;
            this.kUseSmartControl = useSmartControl;
            this.kSyncIntegratedWithAbsoluteOnStartup = syncIntegratedWithAbsoluteOnStartup;
            this.kSyncIntegratedWithAbsoluteRegularly = syncIntegratedWithAbsoluteRegularly;
        }
    }
    private final MotorGroup drive, turn;
    private final FlywheelSim driveSim, turnSim;
    private final SwerveModuleConfiguration config;
    private final SmartAbsoluteEncoder rotateEncoder;
    /**
     * Creates a new SwerveModule.
     * @param drive the motor group responsible for controlling the drive.
     * @param turn the motor group responsible for turning.
     * @param encoder the absolute encoder to be used.
     * @param config the configuration of the swerve module.
     */
    public SwerveModule(MotorGroup drive, MotorGroup turn, SmartAbsoluteEncoder encoder, SwerveModuleConfiguration config)
    {
        this.drive = drive;
        this.turn = turn;
        this.rotateEncoder = encoder;
        this.config = config;
        if (RobotBase.isSimulation())
        {
            driveSim = new FlywheelSim(drive.getGearbox(), config.kDriveGearRatio, 0.15);
            turnSim = new FlywheelSim(turn.getGearbox(), config.kTurnGearRatio, 0.15);
        }
        else
        {
            driveSim = null;
            turnSim = null;
        }
        if (config.kSyncIntegratedWithAbsoluteOnStartup) syncIntegratedWithAbsolute();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", () -> getAngle().getDegrees(), null);
    }
    /**
     * Gets the position of the swerve module
     * @return the distance and angle of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drive.getEncoderRotations() * config.kDriveDistancePerRevolution, getAngle());
    }
    /**
     * Gets the state of the swerve module
     * @return the speed and angle of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(drive.getEncoderRPS() * config.kDriveDistancePerRevolution, getAngle());
    }
    /**
     * Sets the state of the swerve module
     * @param state the target angle and speed. Speed should be [-1 .. 1] for open loop, and in m/s for closed-loop.
     * @param useClosedLoop whether to use the closed loop control of the motor, or to simply apply voltage
     */
    public void setState(SwerveModuleState state, boolean useClosedLoop) {
        double velocity = state.speedMetersPerSecond;
        Rotation2d deltaAngle = state.angle.minus(getAngle());
        deltaAngle = Rotation2d.fromDegrees(MathUtil.inputModulus(deltaAngle.getDegrees(), -180, 180));
        if (Math.abs(deltaAngle.getDegrees()) > 90)
        {
            deltaAngle = Rotation2d.fromDegrees(MathUtil.inputModulus(deltaAngle.getDegrees(), -90, 90));
            velocity = -velocity;
        }
        if (useClosedLoop)
        {
            drive.setVelocity(velocity / config.kDriveDistancePerRevolution);
        }
        else
        {
            drive.set(velocity);
        }
        if (config.kUseSmartControl)
        {
            turn.setSmartPosition(turn.getEncoderRotations() + (deltaAngle.getRotations() * config.kTurnGearRatio));
        }
        else
        {
            turn.setPosition(turn.getEncoderRotations() + (deltaAngle.getRotations() * config.kTurnGearRatio));
        }
    }
    /**
     * Updates the simulation model. Should be called every 20ms in simulationPeriodic()
     */
    public void simulationUpdate() {
        driveSim.setInputVoltage(drive.getSimulationOutputVoltage());
        turnSim.setInputVoltage(turn.getSimulationOutputVoltage());
        driveSim.update(0.02);
        turnSim.update(0.02);
        drive.setSimulationVelocity(driveSim.getAngularVelocityRPM() * config.kDriveGearRatio / 60);
        turn.setSimulationVelocity(turnSim.getAngularVelocityRPM() * config.kTurnGearRatio / 60);
        double deltaTurn = turnSim.getAngularVelocityRPM() * config.kTurnGearRatio / 60 * 0.02;
        double deltaDrive = driveSim.getAngularVelocityRPM() * config.kDriveGearRatio / 60 * 0.02;
        drive.addSimulationPosition(deltaDrive);
        turn.addSimulationPosition(deltaTurn);
        rotateEncoder.addSimulationRotations(turnSim.getAngularVelocityRPM() / 60 * 0.02);
    }
    /**
     * Checks to see if it can sync the integrated sensor with the absolute encoder. Should be called in SwerveDrive.periodic().
     */
    public void update() {
        if (Math.abs(turn.getEncoderRPS() / config.kTurnGearRatio) < config.kEncoderSyncMaxSpeed)
        {
            if (config.kSyncIntegratedWithAbsoluteRegularly) syncIntegratedWithAbsolute();
        }
    }
    /**
     * Gets the current angle of the swerve module
     * @return rotation2d of module. Does not apply any modulus.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(turn.getEncoderRotations() / config.kTurnGearRatio);
    }
    /**
     * Syncs the integrated sensor position with the position recoreded by the absolute sensor
     */
    public void syncIntegratedWithAbsolute() {
        double integratedPosition = turn.getEncoderRotations() / config.kTurnGearRatio;
        integratedPosition = Math.floor(integratedPosition);
        integratedPosition += MathUtil.inputModulus(rotateEncoder.getAbsoluteEncoderRotations(), 0, 1);
        turn.setEncoderRotations(integratedPosition * config.kTurnGearRatio);
    }
}
