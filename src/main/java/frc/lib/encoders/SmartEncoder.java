package frc.lib.encoders;

public interface SmartEncoder {
    public double getEncoderRotations();
    public double getEncoderRPS();
    public void setSimulationRotations(double rotations);
    public void setSimulationRPS(double rps);
    public void addSimulationRotations(double rotations);
    public void resetEncoderRotations(double rotations);
    public boolean isOnRoborio();
}
