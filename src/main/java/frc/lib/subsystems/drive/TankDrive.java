package frc.lib.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.lib.commands.drive.TankDriveMeters;
import frc.lib.commands.drive.TankStop;
import frc.lib.commands.drive.TankWithJoystick;
import frc.lib.gyros.IMU;
import frc.lib.motors.MotorGroup;
import frc.lib.oi.OI;

public class TankDrive extends Drive {
    public static final class TankDriveConfiguration
    {
        private final double kS, kV, kA, wheelCircumference, distancePerRevolution, trackWidth, kB, kZeta, kP, kI, kD, wheelDiameter,
            robotWeight, gearRatio, maxSpeed, maxAccel;
        public TankDriveConfiguration(double kS, double kV, double kA, double wheelDiameter,
            double gearRatio, double trackWidth, double robotWeight, double kB, double kZeta, double kP, double kI, double kD,
            double maxSpeed, double maxAccel)
        {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.wheelDiameter = wheelDiameter;
            this.wheelCircumference = wheelDiameter * Math.PI;
            this.distancePerRevolution = wheelCircumference / gearRatio;
            this.robotWeight = robotWeight;
            this.kB = kB;
            this.kZeta = kZeta;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.trackWidth = trackWidth;
            this.gearRatio = gearRatio;
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
        }
    }
    private final MotorGroup leftSide;
    private final MotorGroup rightSide;
    private final DifferentialDrive robotDrive;
    private final DifferentialDriveKinematics kinematics;
    private final SimpleMotorFeedforward feedforward;
    private double speedProportion = 1.0, rotationSpeedProportion = 0.75;
    private final TankDriveConfiguration config;
    private final IMU imu;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final RamseteController controller;
    private final PIDController leftController, rightController;
    private final Field2d field = new Field2d();
    private final DifferentialDrivetrainSim simulator;
    /** 
     * Creates a new Drivetrain. 
     */
    public TankDrive(MotorGroup leftSide, MotorGroup rightSide, IMU imu, TankDriveConfiguration config)
    {
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.imu = imu;
        this.config = config;
        robotDrive = new DifferentialDrive(leftSide, rightSide);
        kinematics = new DifferentialDriveKinematics(config.trackWidth);
        feedforward = new SimpleMotorFeedforward(config.kS, config.kV, config.kA);
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, imu.getHeading(), getRightDistance(), getLeftDistance(),
            new Pose2d());
        this.controller = new RamseteController(config.kB, config.kZeta);
        this.leftController = new PIDController(config.kP, config.kI, config.kD);
        this.rightController = new PIDController(config.kP, config.kI, config.kD);
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        if (RobotBase.isReal())
        {
            simulator = null;
        }
        else
        {
            simulator = new DifferentialDrivetrainSim(leftSide.getGearbox(), config.gearRatio, 7.15,
                config.robotWeight, config.wheelDiameter / 2, config.trackWidth,
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
        }
    }
    /**
     * Drives the robot forward applying the speed proportions
     * @param speed forward speed [1.0.. -1.0]
     * @param rotation rotational speed [1.0.. -1.0]
     */
    public void drive(double speed, double rotation) {
        robotDrive.arcadeDrive(speed * speedProportion, rotation * rotationSpeedProportion);
    }
    /**
     * Sets the speed proportions
     * @param speedProportion Forward speed proportion
     * @param rotationSpeedProportion Rotational speed proportion
     */
    public void setSpeedProportions(double speedProportion, double rotationSpeedProportion) {
        this.speedProportion = speedProportion;
        this.rotationSpeedProportion = rotationSpeedProportion;
    }
    /**
     * Gets the forward speed proportion
     * @return Forward speed proportion
     */
    public double getSpeedProportion()
    {
        return speedProportion;
    }
    /**
     * Gets the Rotational Speed Proportion
     * @return the rotational speed proportion
     */
    public double getRotationSpeedProportion()
    {
        return rotationSpeedProportion;
    }
    /**
     * Drives using speeds
     * @param speeds in m/s
     */
    public void driveUsingSpeeds(DifferentialDriveWheelSpeeds speeds)
    {
        leftSide.setVelocity(speeds.leftMetersPerSecond / config.distancePerRevolution);
        rightSide.setVelocity(speeds.rightMetersPerSecond / config.distancePerRevolution);
        robotDrive.feed();
    }
    /**
     * Resets the encoder rotations to (0, 0)
     */
    public void resetEncoderRotations() {
        leftSide.resetEncoderRotations();
        rightSide.resetEncoderRotations();
    }
    /**
     * Gets the distance traveled by the left encoder
     * @return left encoder distance in meters
     */
    public double getLeftDistance()
    {
        return leftSide.getEncoderRotations() * config.distancePerRevolution;
    }
    /**
     * Gets the distance traveled by the right encoder
     * @return right encoder distance in meters
     */
    public double getRightDistance()
    {
        return rightSide.getEncoderRotations() * config.distancePerRevolution;
    }
    /**
     * Gets the speed that the wheels are moving at
     * @return DifferentialDriveWheelSpeeds in m/s
     */
    public DifferentialDriveWheelSpeeds getSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(
            leftSide.getEncoderRPS() * config.distancePerRevolution,
            rightSide.getEncoderRPS() * config.distancePerRevolution
        );
    }
    /**
     * Gets the speed that the drivetrain is moving at as vx and vy
     * @return ChassisSpeeds in m/s
     */
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getSpeeds());
    }
    /**
     * Drives the drivetrain based on voltage amounts
     * @param left amount of voltage to go into the left motors
     * @param right amount of voltage to go into the right motors
     */
    public void driveVolts(double left, double right) {
        leftSide.setVoltage(left);
        rightSide.setVoltage(right);
        robotDrive.feed();
    }
    /**
     * Drives using meters per second
     * @param speeds ChassisSpeeds
     */
    @Override
    public void driveUsingChassisSpeeds(ChassisSpeeds speeds, boolean useClosedLoop) {
        if (useClosedLoop)
        {
            driveUsingSpeeds(kinematics.toWheelSpeeds(speeds));
        }
        else
        {
            var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
            driveVolts(
                feedforward.calculate(wheelSpeeds.leftMetersPerSecond),
                feedforward.calculate(wheelSpeeds.rightMetersPerSecond)
            );
        }
    }
    /** 
     * Returns SimpleMotorFeedforward
     * @return feedforward
     */
    public SimpleMotorFeedforward getFeedforward()
    {
        return feedforward;
    }
    /** 
     * Returns DifferentialDriveKinematics
     * @return kinematics
     */
    public DifferentialDriveKinematics getKinematics()
    {
        return kinematics;
    }
    /**
     * Adds a vision measurement
     * @param estimation the vision estimation of pose
     * @param timestamp the timestamp of the estimation
     */
    public void addVisionEstimate(Pose2d estimate, double timestamp)
    {
        poseEstimator.addVisionMeasurement(estimate, timestamp);
    }
    /**
     * Resets the position to a new position
     * @param pose the new position
     */
    @Override
    public void resetPosition(Pose2d pose) {
        imu.setHeading(pose.getRotation());
        poseEstimator.resetPosition(imu.getHeading(), getLeftDistance(), getRightDistance(), pose);
    }
    /**
     * Gets the current estimated pose
     * @return the current pose estimation
     */
    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    @Override
    public void periodic()
    {
        poseEstimator.update(imu.getHeading(), getLeftDistance(), getRightDistance());
        field.setRobotPose(getPose());
    }
    @Override
    public void simulationPeriodic()
    {
        simulator.setInputs(leftSide.getSimulationOutputVoltage(), rightSide.getSimulationOutputVoltage());
        simulator.update(0.02);
        leftSide.setSimulationPosition(simulator.getLeftPositionMeters() / config.distancePerRevolution);
        leftSide.setSimulationVelocity(simulator.getLeftVelocityMetersPerSecond() / config.distancePerRevolution);
        rightSide.setSimulationPosition(simulator.getRightPositionMeters() / config.distancePerRevolution);
        rightSide.setSimulationVelocity(simulator.getRightVelocityMetersPerSecond() / config.distancePerRevolution);
        imu.setSimulationHeading(simulator.getHeading());
    }
    @Override
    public Command getStopCommand()
    {
        return new TankStop(this);
    }
    @Override
    public Command getDriveMetersCommand(double speed, double meters)
    {
        return new TankDriveMeters(this, speed, meters);
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return new TankWithJoystick(this, oi);
    }
    @Override
    public Rotation2d getHeading() {
        return imu.getHeading();
    }
    @Override
    public void resetHeading(Rotation2d newHeading) {
        imu.setHeading(newHeading);
    }
    @Override
    public Command getFollowPathCommand(PathPlannerTrajectory trajectory) {
        return new RamseteCommand(trajectory, this::getPose, controller,
            feedforward, kinematics, this::getSpeeds, leftController, rightController, this::driveVolts, this);
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
        builder.addDoubleProperty("Speed (ft/s)", () -> Units.metersToFeet(
            (getSpeeds().leftMetersPerSecond + getSpeeds().rightMetersPerSecond) / 2
        ), null);
    }
    @Override
    public Field2d getField()
    {
        return field;
    }
    @Override
    public Command getDriveToPointCommand(Pose2d point)
    {
        return null;
    }
    @Override
    public double getMaxSpeed() {
        return config.maxSpeed;
    }
    @Override
    public double getMaxAccel() {
        return config.maxAccel;
    }
}
