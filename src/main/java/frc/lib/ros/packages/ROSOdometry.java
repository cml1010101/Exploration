package frc.lib.ros.packages;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.ros.Coprocessor;
import frc.lib.subsystems.drive.Drive;

public class ROSOdometry implements ROSPackage {
    private final Drive drive;
    private final NetworkTable odomTable;
    private final NetworkTableEntry odomX, odomY, odomTheta, velocityX, velocityY, velocityTheta;
    public ROSOdometry(Coprocessor processor, Drive drive)
    {
        this.drive = drive;
        this.odomTable = processor.getROSTable("odom");
        this.odomX = odomTable.getEntry("x");
        this.odomY = odomTable.getEntry("y");
        this.odomTheta = odomTable.getEntry("theta");
        this.velocityX = odomTable.getEntry("vx");
        this.velocityY = odomTable.getEntry("vy");
        this.velocityTheta = odomTable.getEntry("vtheta");
    }
    public void update()
    {
        odomX.setDouble(drive.getPose().getX());
        odomY.setDouble(drive.getPose().getY());
        odomTheta.setDouble(drive.getPose().getRotation().getRadians());
        velocityX.setDouble(drive.getChassisSpeeds().vxMetersPerSecond);
        velocityY.setDouble(drive.getChassisSpeeds().vyMetersPerSecond);
        velocityTheta.setDouble(drive.getChassisSpeeds().omegaRadiansPerSecond);
    }
}
