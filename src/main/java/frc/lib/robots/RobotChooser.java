package frc.lib.robots;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;
import java.util.function.Supplier;

public class RobotChooser {
    private final Map<String, Supplier<RobotContainer>> robots;
    private final Supplier<RobotContainer> defaultRobot;
    public RobotChooser(Map<String, Supplier<RobotContainer>> robots, Supplier<RobotContainer> defaultRobot)
    {
        this.robots = robots;
        this.defaultRobot = defaultRobot;
    }
    public RobotContainer getRobotContainer()
    {
        File file = new File("/home/admin/robot_settings.txt");
        try
        {
            Scanner scanner = new Scanner(file);
            String robotName = scanner.next();
            scanner.close();
            if (robots.containsKey(robotName))
            {
                return robots.get(robotName).get();
            }
            else
            {
                return defaultRobot.get();
            }
        }
        catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return defaultRobot.get();
    }
}
