package frc.lib.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.SmartSubsystem;

public abstract class Drive extends SmartSubsystem {
    public Drive(ShuffleboardTab tab, String name) {
        super(tab, name);
    }
    /**
     * Gets the speed that the drivetrain is moving at as vx and vy
     * @return ChassisSpeeds in m/s
     */
    public abstract ChassisSpeeds getChassisSpeeds();
    /**
     * Drives using meters per second
     * @param speeds ChassisSpeeds
     * @param useClosedLoop whether to use closed loop velocity control or just percentage of maximum speed
     */
    public abstract void driveUsingChassisSpeeds(ChassisSpeeds speeds, boolean useClosedLoop);
    /**
     * Adds a vision measurement
     * @param estimation the vision estimation of pose
     * @param timestamp the timestamp of the estimation
     */
    public abstract void addVisionEstimate(Pose2d estimate, double timestamp);
    /**
     * Resets the position to a new position
     * @param pose the new position
     */
    public abstract void resetPosition(Pose2d pose);
    /**
     * Gets the current estimated pose
     * @return the current pose estimation
     */
    public abstract Pose2d getPose();
    /**
     * Gets the command to stop the movement of the drivetrain
     * @return a stop command
     */
    public abstract Command getStopCommand();
    /**
     * Gets a command to drive the chassis forward an amount of meters
     * @param speed the speed to drive [-1.. 1]
     * @param meters the amount of meters to drive
     * @return a drivemeters command
     */
    public abstract Command getDriveMetersCommand(double speed, double meters);
    /**
     * Gets the current heading of the robot using the imu
     * @return the current heading
     */
    public abstract Rotation2d getHeading();
    /**
     * Sets the current heading of the robot
     * @param heading the new heading
     */
    public abstract void resetHeading(Rotation2d newHeading);
    /**
     * Gets a command to follow a pathplanner trajectory
     * @return a holonomic drive command
     */
    public abstract Command getFollowPathCommand(PathPlannerTrajectory trajectory);
    /**
     * Returns a field that holds the coordinates of the robot
     * @return field that contains robot location
     */
    public abstract Field2d getField();
    /**
     * Drives the robot to a specified point
     * @param point pose2d
     * @return the command to do so
     */
    public abstract Command getDriveToPointCommand(Pose2d point);
    /**
     * Gets the max speed of the chassis
     * @return max speed in m/s
     */
    public abstract double getMaxSpeed();
    /**
     * Gets the max accel of the chassis
     * @return max accel in m/s^2
     */
    public abstract double getMaxAccel();
}
