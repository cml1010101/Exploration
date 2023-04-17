package frc.lib.robots;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.oi.OI;
import frc.lib.subsystems.SmartSubsystem;
import frc.lib.subsystems.drive.Drive;

public abstract class RobotContainer {
    public RobotContainer()
    {
    }
    public abstract void periodic();
    public abstract Command getAutonomousCommand();
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
}