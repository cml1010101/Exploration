package frc.robot.robots;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.encoders.SmartCANCoder;
import frc.lib.encoders.SmartSparkAbsoluteEncoder;
import frc.lib.gyros.NavX;
import frc.lib.motors.MotorGroupSpark;
import frc.lib.motors.MotorGroupTalonFX;
import frc.lib.oi.OI;
import frc.lib.robots.RobotContainer;
import frc.lib.subsystems.SmartSubsystem;
import frc.lib.subsystems.arm.Arm;
import frc.lib.subsystems.arm.IntakeArmJoint;
import frc.lib.subsystems.arm.RotatingArmJoint;
import frc.lib.subsystems.arm.TelescopingArmJoint;
import frc.lib.subsystems.arm.IntakeArmJoint.IntakeArmJointConfiguration;
import frc.lib.subsystems.arm.RotatingArmJoint.RotatingArmJointConfiguration;
import frc.lib.subsystems.arm.TelescopingArmJoint.TelescopingArmJointConfiguration;
import frc.lib.subsystems.drive.Drive;
import frc.lib.subsystems.drive.TankDrive;
import frc.lib.subsystems.drive.TankDrive.TankDriveConfiguration;
import frc.lib.subsystems.pneumatics.PCM;
import frc.lib.subsystems.pneumatics.PCM.PCMConfiguration;

public class PinkyContainer extends RobotContainer {
    public static class RobotMap
    {
        public static final int LEFT_PRIMARY_ID = 1;
        public static final int RIGHT_PRIMARY_ID = 2;
        public static final int LEFT_SECONDARY_ID = 3;
        public static final int RIGHT_SECONDARY_ID = 4;
        public static final int SHOULDER_PRIMARY_ID = 5;
        public static final int SHOULDER_SECONDARY_ID = 6;
        public static final int SHOULDER_CANCODER_ID = 13;
        public static final int INTAKE_ID = 10;
        public static final int WRIST_ID = 9;
        public static final int PNEUMATICS_ID = 20;
        public static class Solenoids
        {
            public static final int TELESCOPE_ID = 8;
        }
    }
    public static class Constants
    {
        public static final double kRobotWeight = Units.lbsToKilograms(128);
        public static class DriveConstants
        {
            public static final double kS = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;
            public static final double kTrackWidth = Units.inchesToMeters(22);
            public static final double kB = 3.0;
            public static final double kZeta = 0.7;
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kGearRatio = 11.38;
            public static final double kWheelDiameter = Units.inchesToMeters(6);
            public static final double kMaxSpeed = 3.0;
            public static final double kMaxAccel = 9.0;
            public static final double kMaxDriveTicksPer100ms = 10000;
            public static final double kDriveP = 1.0;
            public static final double kDriveI = 0.0;
            public static final double kDriveD = 0.0;
            public static final double kDriveF = 1023 / kMaxDriveTicksPer100ms;
            public static final TankDriveConfiguration kDriveConfiguration = new TankDriveConfiguration(
                kS,
                kV,
                kA,
                kWheelDiameter,
                kGearRatio,
                kTrackWidth,
                kRobotWeight,
                kB,
                kZeta,
                kP,
                kI,
                kD,
                kMaxSpeed,
                kMaxAccel
            );
            public static final TalonFXConfiguration kDriveMotorConfiguration = new TalonFXConfiguration();
            static
            {
                kDriveMotorConfiguration.supplyCurrLimit.currentLimit = 40;
                kDriveMotorConfiguration.supplyCurrLimit.triggerThresholdCurrent = 90;
                kDriveMotorConfiguration.supplyCurrLimit.triggerThresholdTime = 1;
                kDriveMotorConfiguration.slot0.allowableClosedloopError = 0;
                kDriveMotorConfiguration.slot0.kP = kDriveP;
                kDriveMotorConfiguration.slot0.kI = kDriveI;
                kDriveMotorConfiguration.slot0.kD = kDriveD;
                kDriveMotorConfiguration.slot0.kF = kDriveF;
            }
        }
        public static class ShoulderConstants
        {
            public static final double kP = 1.2;
            public static final double kMaxCountsPer100MS = 117.16;
            public static final double kF = (0.85 * 1023) / 117.16;
            public static final double kInitialVelocity = kMaxCountsPer100MS * 0.6;
            public static final double kInitialAcceleration = kInitialVelocity * 5;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kGearRatio = 2200 / 64.0;
            public static final TalonFXConfiguration kShoulderMotorConfiguration = new TalonFXConfiguration();
            static
            {
                kShoulderMotorConfiguration.supplyCurrLimit.currentLimit = 40;
                kShoulderMotorConfiguration.supplyCurrLimit.triggerThresholdCurrent = 90;
                kShoulderMotorConfiguration.supplyCurrLimit.triggerThresholdTime = 1;
                kShoulderMotorConfiguration.slot0.allowableClosedloopError = 0;
                kShoulderMotorConfiguration.slot0.kP = kP;
                kShoulderMotorConfiguration.slot0.kI = kI;
                kShoulderMotorConfiguration.slot0.kD = kD;
                kShoulderMotorConfiguration.slot0.kF = kF;
                kShoulderMotorConfiguration.motionAcceleration = kInitialAcceleration;
                kShoulderMotorConfiguration.motionCruiseVelocity = kInitialVelocity;
            }
        }
        public static class IntakeConstants
        {
            public static final Translation3d kFulcrumOffset = new Translation3d(
                Units.inchesToMeters(10),
                0,
                Units.inchesToMeters(1)
            );
            public static final IntakeArmJointConfiguration kIntakeArmJointConfiguration = new IntakeArmJointConfiguration(
                kFulcrumOffset, new Rotation3d(), 20
            );
        }
        public static class WristConstants
        {
            public static final Translation3d kFulcrumOffset = new Translation3d(
                0,
                Units.inchesToMeters(2),
                0
            );
            public static final RotatingArmJointConfiguration kRotatingArmJointConfiguration = new RotatingArmJointConfiguration(
                40,
                1,
                0,
                Units.lbsToKilograms(5),
                Units.inchesToMeters(15),
                Rotation2d.fromDegrees(-10),
                Rotation2d.fromDegrees(175),
                Rotation2d.fromDegrees(10),
                Rotation2d.fromDegrees(0),
                false,
                true,
                false,
                false,
                0,
                kFulcrumOffset,
                new Rotation3d(),
                false,
                "Wrist"
            );
        }
        public static class Telescope
        {
            public static final TelescopingArmJointConfiguration kTelescopingArmJointConfiguration
                = new TelescopingArmJointConfiguration(Units.inchesToMeters(26), Units.inchesToMeters(40),
                    1.5, 1.5, new Rotation3d(), new Translation3d(), "Telescoping Joint");
        }
        public static class Pneumatics
        {
            public static final PCMConfiguration kPCMConfiguration = new PCMConfiguration(
                PneumaticsModuleType.REVPH, false, 0, 0);
        }
        public static class ArmRotate
        {
            public static final Translation3d kFulcrumOffset = new Translation3d(
                Units.inchesToMeters(3), 0, Units.inchesToMeters(26.5));
            public static final RotatingArmJointConfiguration kRotatingArmJointConfiguration =
                new RotatingArmJointConfiguration(
                    2200 / 64,
                    1,
                    0,
                    Units.lbsToKilograms(20),
                    Units.inchesToMeters(26),
                    Rotation2d.fromDegrees(-45),
                    Rotation2d.fromDegrees(197),
                    Rotation2d.fromDegrees(5),
                    Rotation2d.fromDegrees(90),
                    true,
                    true,
                    false,
                    false,
                    0,
                    kFulcrumOffset,
                    new Rotation3d(),
                    false,
                    "Arm Rotate"
                );
        }
    }
    private final Drive drive;
    private final PCM pcm;
    private final RotatingArmJoint shoulderJoint;
    private final TelescopingArmJoint telescopingJoint;
    private final RotatingArmJoint wristJoint;
    private final IntakeArmJoint intakeJoint;
    private final Arm arm;
    public PinkyContainer()
    {
        MotorGroupTalonFX leftDrive = new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration,
            RobotMap.LEFT_PRIMARY_ID, RobotMap.LEFT_SECONDARY_ID);
        MotorGroupTalonFX rightDrive = new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration,
            RobotMap.RIGHT_PRIMARY_ID, RobotMap.RIGHT_SECONDARY_ID);
        rightDrive.setInverted(true);
        drive = new TankDrive(leftDrive, rightDrive, new NavX(), Constants.DriveConstants.kDriveConfiguration);
        MotorGroupSpark intakeMotor = new MotorGroupSpark(MotorType.kBrushless, () -> DCMotor.getNeo550(1),
            RobotMap.INTAKE_ID);
        intakeJoint = new IntakeArmJoint(intakeMotor, Constants.IntakeConstants.kIntakeArmJointConfiguration);
        MotorGroupSpark wristMotor = new MotorGroupSpark(MotorType.kBrushed, () -> new DCMotor(12,
            0.1, 0.1, 0.1, 50, 1),
            RobotMap.WRIST_ID);
        wristJoint = new RotatingArmJoint(wristMotor, new SmartSparkAbsoluteEncoder(wristMotor.getAbsoluteEncoder()),
            Constants.WristConstants.kRotatingArmJointConfiguration);
        pcm = new PCM(Shuffleboard.getTab("PCM"), RobotMap.PNEUMATICS_ID, Constants.Pneumatics.kPCMConfiguration);
        telescopingJoint = new TelescopingArmJoint(pcm.getSingleSolenoid(RobotMap.Solenoids.TELESCOPE_ID),
            Constants.Telescope.kTelescopingArmJointConfiguration);
        MotorGroupTalonFX shoulderMotor = new MotorGroupTalonFX(Constants.ShoulderConstants.kShoulderMotorConfiguration,
            RobotMap.SHOULDER_PRIMARY_ID, RobotMap.SHOULDER_SECONDARY_ID);
        shoulderJoint = new RotatingArmJoint(shoulderMotor, new SmartCANCoder(RobotMap.SHOULDER_CANCODER_ID),
            Constants.ArmRotate.kRotatingArmJointConfiguration);
        arm = new Arm(
            Shuffleboard.getTab("Arm"),
            shoulderJoint,
            telescopingJoint,
            wristJoint,
            intakeJoint
        );
        Shuffleboard.getTab("General").add("Arm", arm.getMechanism());
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
        return Map.of();
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
