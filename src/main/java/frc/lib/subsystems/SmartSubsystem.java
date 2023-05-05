package frc.lib.subsystems;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.oi.OI;

public abstract class SmartSubsystem extends SubsystemBase {
    protected static boolean isDebugging = false;
    private static final ArrayList<SmartSubsystem> declaredSubsystems = new ArrayList<>();
    private final HashMap<Field, SimpleWidget> debugValues;
    protected final ShuffleboardTab tab;
    public SmartSubsystem(ShuffleboardTab tab, String name)
    {
        declaredSubsystems.add(this);
        this.tab = tab;
        setName(name);
        debugValues = new HashMap<>();
        if (isDebugging) startDebugging();
    }
    protected void startDebugging()
    {
        if (tab == null) return;
        for (var field : getClass().getFields())
        {
            if (field.getAnnotationsByType(DebugValue.class).length > 0)
            {
                SimpleWidget entry = null;
                try
                {
                    if (field.getGenericType() == Double.TYPE)
                        entry = tab.add(field.getName(), field.getDouble(this));
                    else if (field.getGenericType() == Integer.TYPE)
                        entry = tab.add(field.getName(), field.getInt(this));
                    else if (field.getGenericType() == Boolean.TYPE)
                        entry = tab.add(field.getName(), field.getBoolean(this));
                    else if (field.getGenericType() == Long.TYPE)
                        entry = tab.add(field.getName(), field.getLong(this));
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    debugValues.put(field, entry);
                }
            }
        }
    }
    protected void stopDebugging()
    {
        if (tab == null) return;
        for (var field : debugValues.keySet())
        {
            debugValues.get(field).close();
            debugValues.remove(field);
        }
    }
    public abstract Command getDefaultCommand(OI oi);
    public static void enableDebugging()
    {
        if (!isDebugging)
        {
            isDebugging = true;
            for (var subsystem : declaredSubsystems)
            {
                subsystem.startDebugging();
            }
        }
    }
    public static void disableDebugging()
    {
        if (isDebugging)
        {
            isDebugging = false;
            for (var subsystem : declaredSubsystems)
            {
                subsystem.stopDebugging();
            }
        }
    }
    @Override
    public void periodic()
    {
        if (isDebugging && tab != null)
        {
            for (var field : debugValues.keySet())
            {
                try
                {
                    if (field.getGenericType() == Double.TYPE)
                        field.setDouble(this, debugValues.get(field).getEntry().getDouble(field.getDouble(this)));
                    else if (field.getGenericType() == Integer.TYPE)
                        field.setInt(this, (int)debugValues.get(field).getEntry().getInteger(field.getInt(this)));
                    else if (field.getGenericType() == Boolean.TYPE)
                        field.setBoolean(this, debugValues.get(field).getEntry().getBoolean(field.getBoolean(this)));
                    else if (field.getGenericType() == Long.TYPE)
                        field.setLong(this, debugValues.get(field).getEntry().getInteger(field.getLong(this)));
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
