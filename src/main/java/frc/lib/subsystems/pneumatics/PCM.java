package frc.lib.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.oi.OI;
import frc.lib.pneumatics.SmartDoubleSolenoid;
import frc.lib.pneumatics.SmartSingleSolenoid;
import frc.lib.subsystems.SmartSubsystem;

public class PCM extends SmartSubsystem {
    public static final class PCMConfiguration
    {
        private final PneumaticsModuleType type;
        private final boolean hasAnalogSensor;
        private final double minPressure, maxPressure;
        public PCMConfiguration(PneumaticsModuleType type, boolean hasAnalogSensor, double minPressure, double
            maxPressure)
        {
            this.type = type;
            this.hasAnalogSensor = hasAnalogSensor;
            this.minPressure = minPressure;
            this.maxPressure = maxPressure;
        }
    }
    private final PCMConfiguration config;
    private final Compressor compressor;
    private final int id;
    public PCM(ShuffleboardTab tab, int id, PCMConfiguration config)
    {
        this.config = config;
        this.compressor = new Compressor(id, config.type);
        if (config.hasAnalogSensor)
        {
            compressor.enableAnalog(config.minPressure, config.maxPressure);
        }
        else
        {
            compressor.enableDigital();
        }
        this.id = id;
        if (tab != null)
        {
            if (config.hasAnalogSensor)
            {
                tab.addDouble("Pressure", () -> compressor.getPressure());
            }
            tab.addBoolean("Pressurized? ", () -> !compressor.getPressureSwitchValue());
        }
    }
    public PCM(int id, PCMConfiguration config)
    {
        this(null, id, config);
    }
    public SmartSingleSolenoid getSingleSolenoid(int id)
    {
        return new SmartSingleSolenoid(this.id, config.type, id);
    }
    public SmartDoubleSolenoid getDoubleSolenoid(int forwardID, int reverseID)
    {
        return new SmartDoubleSolenoid(this.id, config.type, forwardID, reverseID);
    }
    @Override
    public Command getDefaultCommand(OI oi) {
        return null;
    }
}
