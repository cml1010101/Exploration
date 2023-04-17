package frc.robot.robots;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.gyros.NavX;
import frc.lib.motors.MotorGroupTalonFX;
import frc.lib.oi.OI;
import frc.lib.robots.RobotContainer;
import frc.lib.subsystems.SmartSubsystem;
import frc.lib.subsystems.drive.Drive;
import frc.lib.subsystems.drive.TankDrive;
import frc.lib.subsystems.drive.TankDrive.TankDriveConfiguration;

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
    }
    private final Drive drive;
    //private final RotatingArmJoint shoulderJoint;
    //private final TelescopingArmJoint telescopingJoint;
    //private final RotatingArmJoint wristJoint;
    //private final IntakeArmJoint intakeJoint;
    public PinkyContainer()
    {
        MotorGroupTalonFX leftDrive = new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration,
            RobotMap.LEFT_PRIMARY_ID, RobotMap.LEFT_SECONDARY_ID);
        MotorGroupTalonFX rightDrive = new MotorGroupTalonFX(Constants.DriveConstants.kDriveMotorConfiguration,
            RobotMap.RIGHT_PRIMARY_ID, RobotMap.RIGHT_SECONDARY_ID);
        rightDrive.setInverted(true);
        drive = new TankDrive(leftDrive, rightDrive, new NavX(), Constants.DriveConstants.kDriveConfiguration);
        //MotorGroupTalonFX shoulderMotor = new MotorGroupTalonFX(Constants.ShoulderConstants.kShoulderMotorConfiguration,
        //    RobotMap.SHOULDER_PRIMARY_ID, RobotMap.SHOULDER_SECONDARY_ID);
        /*telescopingJoint = new TelescopingArmJoint(new SmartSingleSolenoid(0, null, 0), null)
        shoulderJoint = new RotatingArmJoint(shoulderMotor, new SmartCANCoder(RobotMap.SHOULDER_CANCODER_ID),
            () -> SingleJointedArmSim.estimateMOI(Math.sqrt(Math.pow(telescopingJoint.getLength(), 2) + Math.pow(wristJoint.getEndPoint().getNorm(), 2) -
            2 * telescopingJoint.getLength() * wristJoint.getEndPoint().getNorm() * Rotation2d.fromDegrees(180).minus(wristJoint.getAngle()).getCos()),
            Units.lbsToKilograms(20)),
            Constants.ShoulderConstants.kShoulderConfiguration
        );*/
    }
    @Override
    public void periodic()
    {
    }
    @Override
    public Command getAutonomousCommand()
    {
        return new InstantCommand();
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
}
