package frc.lib.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SmartSingleSolenoid extends Solenoid implements SmartSolenoid {
    public SmartSingleSolenoid(int module, PneumaticsModuleType moduleType, int channel) {
        super(module, moduleType, channel);
    }
    @Override
    public boolean supportsReverse() {
        return false;
    }
    @Override
    public void setState(SmartSolenoidState state) {
        assert state != SmartSolenoidState.kReverse;
        super.set(state != SmartSolenoidState.kClosed);
    }
    @Override
    public SmartSolenoidState getState() {
        return super.get() ? SmartSolenoidState.kOpen : SmartSolenoidState.kClosed;
    }
}
