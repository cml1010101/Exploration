// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.motors;

import java.util.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.encoders.SmartEncoder;
import frc.lib.encoders.SmartTalonFXIntegratedEncoder;


/** 
 * Group of Talons to be used like MotorController Class
*/
public class MotorGroupTalonFX extends TalonFX implements MotorGroup {
    public static final TalonFXConfiguration kDefaultConfiguration = new TalonFXConfiguration();
    static
    {
        kDefaultConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        kDefaultConfiguration.CurrentLimits.SupplyTimeThreshold = 1;
        kDefaultConfiguration.CurrentLimits.SupplyCurrentThreshold = 90;
        kDefaultConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    }
    private List<TalonFX> followers = new ArrayList<TalonFX>();
    private int invertCoefficient = 1;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupTalonFX(int primaryID) {
        this(primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons.
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupTalonFX(int primaryID, int... followerIDs) {
        this(kDefaultConfiguration, primaryID, followerIDs);
    }
    public MotorGroupTalonFX(TalonFXConfiguration configuration, int primaryID)
    {
        this(configuration, primaryID, new int[]{});
    }
    public MotorGroupTalonFX(TalonFXConfiguration configuration, int primaryID, int... followerIDs)
    {
        super(primaryID);
        for (int followerID: followerIDs) {
            this.followers.add(new TalonFX(followerID));
        }
        setFollowers();
        setInverted(false);
        configureAllControllers(configuration);
    }
    public double getEncoderRotations() {
        return getPosition().getValue();
    }
    public double getEncoderRPS() {
        return getVelocity().getValue();
    }
    /**
     * Sets the position of Falcon motor using integrated PIDControl
     * @param rotations Number of rotations. Does not factor in gear ratio.
     */
    @Override
    public void setPosition(int slot, double rotations, double feedforward)
    {
        PositionVoltage pv = new PositionVoltage(rotations);
        pv.FeedForward = feedforward;
        pv.Slot = slot;
    }
    @Override
    public void setPercent(double percent, double feedforward)
    {
        set(percent + feedforward);
    }
    @Override
    public void setInverted(boolean inverted)
    {
        super.setInverted(inverted);
        invertCoefficient = inverted ? -1 : 1;
    }
    /*
     * Sets the position using motion magic
     * @param position position in rotations... does not factor in gear ratio
     * @param feedforward the feedforward value to be added
     */
    public void setMotionMagic(double position, double feedforward)
    { 
        set(ControlMode.MotionMagic, position * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
    }
    public void setFollowerOppose(int i) {
        followers.get(i).setInverted(InvertType.OpposeMaster);
    }
    public void resetEncoderRotations() {
        setSelectedSensorPosition(0);
    }
    /**
     * Links and selects the cancoder
     * @param coder the CANCoder reference
     */
    public void linkAndUseCANCoder(CANCoder coder)
    {
        configRemoteFeedbackFilter(coder, 0);
        configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        COUNTS_PER_REVOLUTION = 4096;
    }
    public void configSelectedProfile(int slotIdx, int pidIdx)
    {
        selectProfileSlot(slotIdx, pidIdx);
    }
    public void configureAllControllers(TalonFXConfiguration configuration) {
        configureController(this, configuration);
        for (WPI_TalonFX wpi_TalonFX : followers) {
            configureController(wpi_TalonFX, configuration);
        }
    }
    private void configureController(WPI_TalonFX controller, TalonFXConfiguration configuration) {
        controller.configAllSettings(configuration);
    }
    private void setFollowers() {
        for (WPI_TalonFX wpi_TalonFX : followers) {
            wpi_TalonFX.follow(this);
            wpi_TalonFX.setInverted(InvertType.FollowMaster);
        }
    }
    /**
     * Gets iteratable of all of the motors, including the primary.
     */
    public List<WPI_TalonFX> getAllMotors()
    {
        ArrayList<WPI_TalonFX> list = new ArrayList<WPI_TalonFX>();
        list.add(this);
        for (var follower : followers)
        {
            list.add(follower);
        }
        return list;
    }
    public void setLimits(Rotation2d positive, Rotation2d negative) {
        configForwardSoftLimitThreshold(positive.getRotations() * COUNTS_PER_REVOLUTION, 0);
        configReverseSoftLimitThreshold(negative.getRotations() * COUNTS_PER_REVOLUTION, 0);
        configForwardSoftLimitEnable(true, 0);
        configReverseSoftLimitEnable(true, 0);
    }
    @Override
    public void setVelocity(double velocity) {
        SmartDashboard.putNumber(getDeviceID() + ": velocity", velocity);
        set(ControlMode.Velocity, velocity * COUNTS_PER_REVOLUTION / 10);
    }
    public double getSimulationOutputVoltage()
    {
        return simData.getMotorOutputLeadVoltage() * invertCoefficient;
    }
    public void setSimulationPosition(double position)
    {
        simData.setIntegratedSensorRawPosition((int) (COUNTS_PER_REVOLUTION * position * invertCoefficient));
    }
    public void setSimulationVelocity(double velocity)
    {
        simData.setIntegratedSensorVelocity((int) (COUNTS_PER_REVOLUTION * velocity * invertCoefficient / 10));
    }
    public void addSimulationPosition(double deltaPosition)
    {
        simData.addIntegratedSensorPosition((int)(deltaPosition * COUNTS_PER_REVOLUTION * invertCoefficient));
    }
    @Override
    public void setPosition(double position) {
        set(ControlMode.Position, position * COUNTS_PER_REVOLUTION);
    }
    @Override
    public DCMotor getGearbox()
    {
        return DCMotor.getFalcon500(1 + followers.size());
    }
    @Override
    public void setEncoderRotations(double rotations) {
        setSelectedSensorPosition(rotations * COUNTS_PER_REVOLUTION);
    }
    @Override
    public void setSmartPosition(double position) {
        setMotionMagic(position, 0);
    }
    @Override
    public void configureClosedLoop(int slotIdx, double kP, double kI, double kD, double kF, double kMaxAcceleration,
        double kMaxVelocity, double kAllowableError) {
        config_kP(slotIdx, kP);
        config_kI(slotIdx, kI);
        config_kD(slotIdx, kD);
        config_kF(slotIdx, kF);
        configAllowableClosedloopError(slotIdx, kAllowableError * COUNTS_PER_REVOLUTION / 10);
        configMotionAcceleration(kMaxAcceleration);
        configMotionCruiseVelocity(kMaxVelocity);
    }
    @Override
    public void setForwardSoftLimit(double rotations)
    {
        configForwardSoftLimitThreshold(rotations * COUNTS_PER_REVOLUTION);
        configForwardSoftLimitEnable(true);
    }
    @Override
    public void setReverseSoftLimit(double rotations)
    {
        configReverseSoftLimitThreshold(rotations * COUNTS_PER_REVOLUTION);
        configReverseSoftLimitEnable(true);
    }
    @Override
    public void disableForwardSoftLimit()
    {
        configForwardSoftLimitEnable(false);
    }
    @Override
    public void disableReverseSoftLimit()
    {
        configReverseSoftLimitEnable(false);
    }
    @Override
    public void linkEncoder(SmartEncoder encoder) throws MotorEncoderMismatchException
    {
        if (SmartTalonFXIntegratedEncoder.class.isAssignableFrom(encoder.getClass()))
        {
            configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            COUNTS_PER_REVOLUTION = 2048;
        }
        else if (CANCoder.class.isAssignableFrom(encoder.getClass()))
        {
            linkAndUseCANCoder((CANCoder)encoder);
        }
        else
        {
            throw new MotorEncoderMismatchException(getClass(), encoder.getClass());
        }
    }
    @Override
    public SmartEncoder getIntegratedEncoder() {
        return new SmartTalonFXIntegratedEncoder(getTalonFXSensorCollection(), this, simData);
    }
    @Override
    public void enableContinuousInput(boolean enable) {
        configFeedbackNotContinuous(!enable, 0);
    }
}
