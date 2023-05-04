package frc.lib.robots;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.oi.OI;
import frc.lib.subsystems.SmartSubsystem;
import frc.lib.subsystems.drive.Drive;

public abstract class RobotContainer {
    public RobotContainer()
    {
    }
    public abstract void periodic();
    public abstract void autonomousPeriodic();
    public abstract Map<String, Pair<Command, Pose2d>> getAutonomousOptions();
    public abstract Pair<String, Pair<Command, Pose2d>> getDefaultOption();
    public abstract List<SmartSubsystem> getAllSubsystems();
    public abstract Drive getDrive();
    public void initializeDefaultCommands(OI oi)
    {
        for (var subsystem : getAllSubsystems())
        {
            Command command = subsystem.getDefaultCommand(oi);
            if (command != null)
            {
                subsystem.setDefaultCommand(command);
            }
        }
    }
    public abstract void bindButtons(OI oi);
    public abstract void pollCamerasPeriodic();
    public abstract void loadStartingPosition(Pose2d selected);
}