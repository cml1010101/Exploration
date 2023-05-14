package frc.lib.ros;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.ros.packages.ROSPackage;

public class Coprocessor {
    private final NetworkTable coprocessorTable;
    private final ArrayList<ROSPackage> packages;
    public Coprocessor(String name)
    {
        coprocessorTable = NetworkTableInstance.getDefault().getTable(name);
        packages = new ArrayList<>();
    }
    public String getStatus()
    {
        return coprocessorTable.getEntry("status").getString("nonexistant");
    }
    public NetworkTable getROSTable(String name)
    {
        return coprocessorTable.getSubTable("ROS").getSubTable(name);
    }
    public void registerPackage(ROSPackage pkg)
    {
        packages.add(pkg);
    }
    public void update()
    {
        packages.forEach(pkg -> pkg.update());
    }
}
