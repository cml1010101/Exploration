package frc.robot.robots;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.lib.motors.MotorGroupTalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.encoders.SmartCANCoder;
import frc.lib.gyros.NavX;
import frc.lib.oi.OI;
import frc.lib.robots.RobotContainer;
import frc.lib.subsystems.SmartSubsystem;
import frc.lib.subsystems.drive.Drive;
import frc.lib.subsystems.drive.SwerveDrive;
import frc.lib.subsystems.drive.SwerveDrive.SwerveDriveConfiguration;
import frc.lib.subsystems.drive.swerve.SwerveModule;
import frc.lib.subsystems.drive.swerve.SwerveModule.SwerveModuleConfiguration;

public class SwervyContainer extends RobotContainer {
    public static class RobotMap
    {
        public static final int LEFT_FORWARD_DRIVE = 1;
        public static final int RIGHT_FORWARD_DRIVE = 2;
        public static final int LEFT_BACKWARD_DRIVE = 3;
        public static final int RIGHT_BACKWARD_DRIVE = 4;
        public static final int LEFT_FORWARD_TURN = 5;
        public static final int RIGHT_FORWARD_TURN = 6;
        public static final int LEFT_BACKWARD_TURN = 7;
        public static final int RIGHT_BACKWARD_TURN = 8;
        public static final int LEFT_FORWARD_CANCODER = 9;
        public static final int RIGHT_FORWARD_CANCODER = 10;
        public static final int LEFT_BACKWARD_CANCODER = 11;
        public static final int RIGHT_BACKWARD_CANCODER = 12;
    }
    public static class Constants
    {
        public static class DriveConstants
        {
            public static final double kMaxAngularTicksPer100ms = 10000;
            public static final double kAngularInitialVelocity = 0.9 * kMaxAngularTicksPer100ms;
            public static final double kAngularInitialAcceleration = 0.9 * kMaxAngularTicksPer100ms;
            public static final double kAngularP = 1.0;
            public static final double kAngularI = 0.0;
            public static final double kAngularD = 0.0;
            public static final double kAngularF = 1023 / kMaxAngularTicksPer100ms;
            public static final double kAngularGearRatio = 12.8;
            public static final double kMaxDriveTicksPer100ms = 10000;
            public static final double kDriveP = 1.0;
            public static final double kDriveI = 0.0;
            public static final double kDriveD = 0.0;
            public static final double kDriveF = 1023 / kMaxDriveTicksPer100ms;
            public static final double kDriveGearRatio = 8.14;
            public static final double kWheelDiameter = Units.inchesToMeters(6);
            public static final double kLinearP = 5.0;
            public static final double kLinearI = 0.0;
            public static final double kLinearD = 0.0;
            public static final double kTurnP = 4.0;
            public static final double kTurnI = 0.0;
            public static final double kTurnD = 0.0;
            public static final double kMaxTurnSpeed = 3.0;
            public static final double kMaxTurnAccel = 6.0;
            public static final double kTrackWidth = 0.581025;
            public static final double kTrackLength = 0.581025;
            public static final boolean kUseSmartControl = true;
            public static final boolean kSyncIntegratedWithAbsoluteOnStartup = true;
            public static final boolean kSyncIntegratedWithAbsoluteRegularly = true;
            public static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration();
            static
            {
                kDriveMotorConfiguration.supplyCurrLimit.currentLimit = 40;
                kDriveMotorConfiguration.supplyCurrLimit.triggerThresholdCurrent = 90;
                kDriveMotorConfiguration.supplyCurrLimit.triggerThresholdTime = 1;
                kDriveMotorConfiguration.slot0.allowableClosedloopError = 0;
                kDriveMotorConfiguration.slot0.kF = Constants.DriveConstants.kDriveF;
                kDriveMotorConfiguration.slot0.kP = Constants.DriveConstants.kDriveP;
                kDriveMotorConfiguration.slot0.kI = Constants.DriveConstants.kDriveI;
                kDriveMotorConfiguration.slot0.kD = Constants.DriveConstants.kDriveD;
            }
            public static final TalonFXConfiguration kTurnMotorConfiguration = new TalonFXConfiguration();
            static
            {
                kTurnMotorConfiguration.supplyCurrLimit.currentLimit = 40;
                kTurnMotorConfiguration.supplyCurrLimit.triggerThresholdCurrent = 90;
                kTurnMotorConfiguration.supplyCurrLimit.triggerThresholdTime = 1;
                kTurnMotorConfiguration.slot0.allowableClosedloopError = 0;
                kTurnMotorConfiguration.slot0.kF = Constants.DriveConstants.kAngularF;
                kTurnMotorConfiguration.slot0.kP = Constants.DriveConstants.kAngularP;
                kTurnMotorConfiguration.slot0.kI = Constants.DriveConstants.kAngularI;
                kTurnMotorConfiguration.slot0.kD = Constants.DriveConstants.kAngularD;
                kTurnMotorConfiguration.motionCruiseVelocity = Constants.DriveConstants.kAngularInitialVelocity;
                kTurnMotorConfiguration.motionAcceleration = Constants.DriveConstants.kAngularInitialAcceleration;
            }
            public static final double kMaxSpeed = 3.0;
            public static final double kMaxAccel = 9.0;
            public static final Pose2d kTolerance = new Pose2d(
                0.1,
                0.1,
                Rotation2d.fromDegrees(5)
            );
            public static final double kEncoderSyncMaxSpeed = 0.1;
            public static final SwerveModuleConfiguration kSwerveModuleConfiguration
                = new SwerveModuleConfiguration(kDriveGearRatio, kAngularGearRatio, kWheelDiameter, kEncoderSyncMaxSpeed,
                    kUseSmartControl, kSyncIntegratedWithAbsoluteRegularly, kSyncIntegratedWithAbsoluteOnStartup);
            public static final SwerveDriveConfiguration kSwerveDriveConfiguration = new SwerveDriveConfiguration(
                kMaxSpeed,
                kMaxAccel,
                kLinearP,
                kLinearI,
                kLinearD,
                kTurnP,
                kTurnI,
                kTurnD,
                kMaxTurnSpeed,
                kMaxTurnAccel,
                kTolerance,
                kTrackWidth,
                kTrackLength
            );
        }
    }
    private final Drive drive;
    public SwervyContainer()
    {
        
        SwerveModule[] modules = new SwerveModule[]
        {
            new SwerveModule(
                new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration, RobotMap.LEFT_FORWARD_DRIVE),
                new MotorGroupTalonFX(Constants.DriveConstants.kTurnMotorConfiguration, RobotMap.RIGHT_FORWARD_TURN),
                new SmartCANCoder(RobotMap.LEFT_FORWARD_CANCODER), Constants.DriveConstants.kSwerveModuleConfiguration
            ),
            new SwerveModule(
                new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration, RobotMap.RIGHT_FORWARD_DRIVE),
                new MotorGroupTalonFX(Constants.DriveConstants.kTurnMotorConfiguration, RobotMap.RIGHT_FORWARD_TURN),
                new SmartCANCoder(RobotMap.RIGHT_FORWARD_CANCODER), Constants.DriveConstants.kSwerveModuleConfiguration
            ),
            new SwerveModule(
                new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration, RobotMap.LEFT_BACKWARD_DRIVE),
                new MotorGroupTalonFX(Constants.DriveConstants.kTurnMotorConfiguration, RobotMap.LEFT_BACKWARD_TURN),
                new SmartCANCoder(RobotMap.LEFT_BACKWARD_CANCODER), Constants.DriveConstants.kSwerveModuleConfiguration
            ),
            new SwerveModule(
                new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration, RobotMap.RIGHT_BACKWARD_DRIVE),
                new MotorGroupTalonFX(Constants.DriveConstants.kTurnMotorConfiguration, RobotMap.RIGHT_BACKWARD_TURN),
                new SmartCANCoder(RobotMap.RIGHT_BACKWARD_CANCODER), Constants.DriveConstants.kSwerveModuleConfiguration
            )
        };
        drive = new SwerveDrive(
            Shuffleboard.getTab("SwerveDrive"),
            modules,
            new NavX(),
            Constants.DriveConstants.kSwerveDriveConfiguration
        );
    }
    @Override
    public void periodic()
    {
    }
    @Override
    public List<SmartSubsystem> getAllSubsystems()
    {
        return List.of(drive);
    }
    @Override
    public Drive getDrive()
    {
        return drive;
    }
    @Override
    public void bindButtons(OI oi) {
    }
    @Override
    public void pollCamerasPeriodic()
    {
    }
    @Override
    public Map<String, Pair<Command, Pose2d>> getAutonomousOptions() {
        return null;
    }
    @Override
    public void loadStartingPosition(Pose2d selected) {
        drive.resetPosition(selected);
        drive.resetHeading(selected.getRotation());
    }
    @Override
    public Pair<String, Pair<Command, Pose2d>> getDefaultOption()
    {
        return new Pair<>("Do Nothing", new Pair<>(null, new Pose2d()));
    }
}
