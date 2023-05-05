package frc.lib.subsystems.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.commands.drive.SmartSwerveControllerCommand;
import frc.lib.commands.drive.SwerveDriveMeters;
import frc.lib.commands.drive.SwerveStop;
import frc.lib.commands.drive.SwerveToPoint;
import frc.lib.commands.drive.SwerveWithJoystick;
import frc.lib.gyros.IMU;
import frc.lib.oi.OI;
import frc.lib.subsystems.drive.swerve.SwerveModule;

public class SwerveDrive extends Drive {
    public static final class SwerveDriveConfiguration
    {
        private final double maxSpeed, linearP, linearI, linearD, angularP, angularI, angularD, maxAngularSpeed, maxAngularAccel,
            maxAccel;
        private final Pose2d tolerance;
        private final Translation2d[] moduleOffsets;
        /**
         * Creates a new SwerveDriveConfiguration.
         * @param maxSpeed of the robot in m/s.
         * @param maxAccel of the robot in m/s^2.
         * @param linearP proportion for drive along path / drive to point.
         * @param linearI integral for drive along path / drive to point.
         * @param linearD derivative for drive along path / drive to point.
         * @param angularP proportion for drive along path / drive to point.
         * @param angularI integral for drive along path / drive to point.
         * @param angularD derivative for drive along path / drive to point.
         * @param maxAngularSpeed max turn speed for drive along path / drive to point.
         * @param maxAngularAccel max turn accel for drive along path / drive to point.
         * @param tolerance for drive along path / drive to point
         * @param moduleOffsets module offsets in meters (front left, front right, back left, back right)
         */
        public SwerveDriveConfiguration(double maxSpeed, double maxAccel, double linearP, double linearI, double linearD,
            double angularP, double angularI, double angularD, double maxAngularSpeed, double maxAngularAccel, Pose2d tolerance,
            Translation2d[] moduleOffsets)
        {
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
            this.linearP = linearP;
            this.linearI = linearI;
            this.linearD = linearD;
            this.angularP = angularP;
            this.angularI = angularI;
            this.angularD = angularD;
            this.maxAngularSpeed = maxAngularSpeed;
            this.maxAngularAccel = maxAngularAccel;
            this.tolerance = tolerance;
            this.moduleOffsets = moduleOffsets;
        }
        /**
         * Creates a new SwerveDriveConfiguration.
         * @param maxSpeed of the robot in m/s.
         * @param maxAccel of the robot in m/s^2.
         * @param linearP proportion for drive along path / drive to point.
         * @param linearI integral for drive along path / drive to point.
         * @param linearD derivative for drive along path / drive to point.
         * @param angularP proportion for drive along path / drive to point.
         * @param angularI integral for drive along path / drive to point.
         * @param angularD derivative for drive along path / drive to point.
         * @param maxAngularSpeed max turn speed for drive along path / drive to point.
         * @param maxAngularAccel max turn accel for drive along path / drive to point.
         * @param tolerance for drive along path / drive to point
         * @param trackWidth width of the track in meters (from left module to right module)
         * @param trackLength height of the track in meters (from back module to front module)
         */
        public SwerveDriveConfiguration(double maxSpeed, double maxAccel, double linearP, double linearI, double linearD,
            double angularP, double angularI, double angularD, double maxAngularSpeed, double maxAngularAccel, Pose2d tolerance,
            double trackWidth, double trackLength)
        {
            this.maxSpeed = maxSpeed;
            this.maxAccel = maxAccel;
            this.linearP = linearP;
            this.linearI = linearI;
            this.linearD = linearD;
            this.angularP = angularP;
            this.angularI = angularI;
            this.angularD = angularD;
            this.maxAngularSpeed = maxAngularSpeed;
            this.maxAngularAccel = maxAngularAccel;
            this.tolerance = tolerance;
            this.moduleOffsets = new Translation2d[]
            {
                new Translation2d(trackLength / 2, trackWidth / 2),
                new Translation2d(trackLength / 2, -trackWidth / 2),
                new Translation2d(-trackLength / 2, trackWidth / 2),
                new Translation2d(-trackLength / 2, -trackWidth / 2),
            };
        }
    };
    private final SwerveDriveConfiguration config;
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final IMU imu;
    private final SwerveDrivePoseEstimator estimator;
    private final HolonomicDriveController controller;
    private final Field2d field;
    private final ShuffleboardTab tab;
    /**
     * Creates a new swerve drive
     * @param tab the shuffleboard tab to displays swerve data in
     * @param modules array of swerve modules (front left, front right, back left, back right)
     * @param imu the gyroscope to be used
     * @param config the SwerveDriveConfiguration to be used
     */
    public SwerveDrive(String name, ShuffleboardTab tab, SwerveModule[] modules, IMU imu, SwerveDriveConfiguration config)
    {
        super(tab, name);
        this.config = config;
        this.modules = modules;
        this.imu = imu;
        this.kinematics = new SwerveDriveKinematics(config.moduleOffsets);
        this.tab = tab;
        imu.reset();
        estimator = new SwerveDrivePoseEstimator(kinematics, imu.getHeading(), getPositions(), new Pose2d());
        controller = new HolonomicDriveController(
            new PIDController(config.linearP, config.linearI, config.linearD),
            new PIDController(config.linearP, config.linearI, config.linearD),
            new ProfiledPIDController(config.angularP, config.angularI, config.angularD,
                new Constraints(config.maxAngularSpeed, config.maxAngularAccel))
        );
        field = new Field2d();
        if (this.tab != null)
        {
            this.tab.addNumber("Front Left", () -> this.modules[0].getAngle().getDegrees())
                .withPosition(0, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kGyro);
            this.tab.addNumber("Front Right", () -> this.modules[1].getAngle().getDegrees())
                .withPosition(2, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kGyro);
            this.tab.addNumber("Back Left", () -> this.modules[2].getAngle().getDegrees())
                .withPosition(4, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kGyro);
            this.tab.addNumber("Back Right", () -> this.modules[3].getAngle().getDegrees())
                .withPosition(6, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kGyro);
            this.tab.add("Front Left Module", this.modules[0])
                .withPosition(0, 2)
                .withSize(2, 3);
            this.tab.add("Front Right Module", this.modules[1])
                .withPosition(2, 2)
                .withSize(2, 3);
            this.tab.add("Back Left Module", this.modules[2])
                .withPosition(4, 2)
                .withSize(2, 3);
            this.tab.add("Back Right Module", this.modules[3])
                .withPosition(6, 2)
                .withSize(2, 3);
        }
        controller.setTolerance(config.tolerance);
    }
    /**
     * Creates a new swerve drive
     * @param modules array of swerve modules (front left, front right, back left, back right)
     * @param imu the gyroscope to be used
     * @param config the SwerveDriveConfiguration to be used
     */
    public SwerveDrive(String name, SwerveModule[] modules, IMU imu, SwerveDriveConfiguration config)
    {
        this(name, null, modules, imu, config);
    }
    /**
     * Gets the speed that the drivetrain is moving at as vx and vy
     * @return ChassisSpeeds in m/s
     */
    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
    }
    /**
     * Drives using meters per second
     * @param speeds ChassisSpeeds
     */
    @Override
    public void driveUsingChassisSpeeds(ChassisSpeeds speeds, boolean useClosedLoop) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        if (!useClosedLoop)
        {
            for (var state : states)
            {
                state.speedMetersPerSecond /= config.maxSpeed;
            }
        }
        modules[0].setState(states[0], useClosedLoop);
        modules[1].setState(states[1], useClosedLoop);
        modules[2].setState(states[2], useClosedLoop);
        modules[3].setState(states[3], useClosedLoop);
    }
    /**
     * Returns the current positions reported by each swerve module.
     * @return (front left, front right, back left, back right)
     */
    public SwerveModulePosition[] getPositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = modules[0].getPosition();
        positions[1] = modules[1].getPosition();
        positions[2] = modules[2].getPosition();
        positions[3] = modules[3].getPosition();
        return positions;
    }
    /**
     * Adds a vision measurement
     * @param estimation the vision estimation of pose
     * @param timestamp the timestamp of the estimation
     */
    @Override
    public void addVisionEstimate(Pose2d estimate, double timestamp) {
        estimator.addVisionMeasurement(estimate, timestamp);
    }
    /**
     * Resets the position to a new position
     * @param pose the new position
     */
    @Override
    public void resetPosition(Pose2d pose) {
        imu.setHeading(pose.getRotation());
        estimator.resetPosition(imu.getHeading(), getPositions(), pose);
    }
    /**
     * Gets the current estimated pose
     * @return the current pose estimation
     */
    @Override
    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return new SwerveWithJoystick(this, oi, oi.isFieldRelative(), config.maxSpeed);
    }
    /**
     * Gets the command to stop the movement of the drivetrain
     * @return a stop command
     */
    @Override
    public Command getStopCommand() {
        return new SwerveStop(this);
    }
    /**
     * Gets a command to drive the chassis forward an amount of meters
     * @param speed the speed to drive [-1.. 1]
     * @param meters the amount of meters to drive
     * @return a drivemeters command
     */
    @Override
    public Command getDriveMetersCommand(double speed, double meters) {
        return new SwerveDriveMeters(this, speed, meters);
    }
    public void setSwerveStates(SwerveModuleState[] states)
    {
        modules[0].setState(states[0], true);
        modules[1].setState(states[1], true);
        modules[2].setState(states[2], true);
        modules[3].setState(states[3], true);
    }
    public SwerveModuleState[] getSwerveStates()
    {
        return new SwerveModuleState[]
        {
            modules[0].getState(),
            modules[1].getState(),
            modules[2].getState(),
            modules[3].getState()
        };
    }
    /**
     * Gets the max speed of the chassis
     * @return max speed in m/s
     */
    @Override
    public double getMaxSpeed()
    {
        return config.maxSpeed;
    }
    /**
     * Gets the max accel of the chassis
     * @return max accel in m/s^2
     */
    @Override
    public double getMaxAccel()
    {
        return config.maxAccel;
    }
    /**
     * Gets the current heading of the robot using the imu
     * @return the current heading
     */
    @Override
    public Rotation2d getHeading() {
        return imu.getHeading();
    }
    /**
     * Sets the current heading of the robot
     * @param heading the new heading
     */
    @Override
    public void resetHeading(Rotation2d newHeading) {
        imu.setHeading(newHeading);
    }
    /**
     * Gets a command to follow a pathplanner trajectory
     * @return a holonomic drive command
     */
    @Override
    public Command getFollowPathCommand(PathPlannerTrajectory trajectory) {
        return new SmartSwerveControllerCommand(trajectory, this::getPose, kinematics, controller, this::setSwerveStates, this);
    }
    @Override
    public void periodic()
    {
        estimator.update(imu.getHeading(), getPositions());
        field.setRobotPose(getPose());
        for (var swerveModule : modules)
        {
            swerveModule.update();
        }
    }
    @Override
    public void simulationPeriodic()
    {
        for (var swerveModule : modules)
        {
            swerveModule.simulationUpdate();
        }
        imu.setSimulationHeading(Rotation2d.fromDegrees(
            Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond) * 0.02 + imu.getHeading().getDegrees()
        ));
    }
    /**
     * Returns a field that holds the coordinates of the robot
     * @return field that contains robot location
     */
    @Override
    public Field2d getField()
    {
        return field;
    }
    /**
     * Gets the holonomic drive controller
     * @return the holonmic drive controller that is used by the trajectory following commands
     */
    public HolonomicDriveController getController()
    {
        return controller;
    }
    /**
     * Drives the robot to a specified point
     * @param point pose2d
     * @return the command to do so
     */
    @Override
    public Command getDriveToPointCommand(Pose2d point)
    {
        return new SwerveToPoint(this, point, new PathConstraints(config.maxSpeed, config.maxAccel));
    }
}
