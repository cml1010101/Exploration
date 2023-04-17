package frc.lib.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SmartDoubleSolenoid extends DoubleSolenoid implements SmartSolenoid {
    public SmartDoubleSolenoid(int module, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        super(module, moduleType, forwardChannel, reverseChannel);
    }
    @Override
    public boolean supportsReverse() {
        return true;
    }
    @Override
    public void setState(SmartSolenoidState state) {
        switch (state)
        {
        case kOpen:
            set(Value.kForward);
            break;
        case kClosed:
            set(Value.kOff);
            break;
        case kReverse:
            set(Value.kReverse);
            break;
        }
    }
    @Override
    public SmartSolenoidState getState() {
        switch (get())
        {
        case kForward:
            return SmartSolenoidState.kOpen;
        case kOff:
            return SmartSolenoidState.kClosed;
        case kReverse:
            return SmartSolenoidState.kReverse;
        }
        return SmartSolenoidState.kClosed;
    }
}
