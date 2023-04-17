package frc.lib.encoders;

public interface SmartAbsoluteEncoder extends SmartEncoder {
    public double getAbsoluteEncoderRotations();
    public double getAbsoluteEncoderOffsetRotations();
    public void setAbsoluteEncoderOffsetRotations(double offset);
    public default void resetAbsoluteEncoderOffset(double newPos)
    {
        setAbsoluteEncoderOffsetRotations(newPos - getAbsoluteEncoderRotations() + getAbsoluteEncoderOffsetRotations());
    }
}
