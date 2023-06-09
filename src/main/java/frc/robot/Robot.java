// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.InvocationTargetException;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.oi.OI;
import frc.lib.robots.RobotChooser;
import frc.lib.robots.RobotContainer;
import frc.robot.oi.DefaultOI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotChooser chooser = ChargedUp.robotChooser;
  private RobotContainer container;
  private Command autonomousCommand = null;
  private OI oi;
  private final double cameraPollTime = 0.04;
  private SendableChooser<Pair<Command, Pose2d>> autoChooser;
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    container = chooser.getRobotContainer();
    loadOI();
    container.initializeDefaultCommands(oi);
    container.bindButtons(oi);
    Shuffleboard.getTab("General").add("Field", container.getDrive().getField());
    autoChooser = new SendableChooser<>();
    for (var option : container.getAutonomousOptions().entrySet())
    {
      autoChooser.addOption(option.getKey(), option.getValue());
    }
    autoChooser.setDefaultOption(container.getDefaultOption().getFirst(), container.getDefaultOption().getSecond());
    Shuffleboard.getTab("General").add("Autonomous Chooser", autoChooser);
    addPeriodic(container::pollCamerasPeriodic, cameraPollTime);
  }
  @Override
  public void endCompetition()
  {
    super.endCompetition();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    container.periodic();
  }

  @Override
  public void autonomousInit() {
    if (autoChooser.getSelected() == null) return;
    autonomousCommand = autoChooser.getSelected().getFirst();
    container.loadStartingPosition(autoChooser.getSelected().getSecond());
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    container.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public void loadOI() {
    String oiName = Preferences.getString("OI", DefaultOI.class.getName());
    try {
      oi = (OI)Class.forName(oiName).getConstructor(new Class<?>[0]).newInstance(new Object[0]);
    } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
      | NoSuchMethodException | SecurityException | ClassNotFoundException e) {
      e.printStackTrace();
      oi = new DefaultOI();
    }
  }
}
