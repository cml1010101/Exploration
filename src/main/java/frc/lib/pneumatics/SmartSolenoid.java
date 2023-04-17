package frc.lib.pneumatics;

public interface SmartSolenoid {
    public static enum SmartSolenoidState
    {
        kOpen,
        kClosed,
        kReverse
    }
    public boolean supportsReverse();
    public void setState(SmartSolenoidState state);
    public SmartSolenoidState getState();
}
