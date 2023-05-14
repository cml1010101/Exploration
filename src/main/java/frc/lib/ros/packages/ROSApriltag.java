package frc.lib.ros.packages;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.ros.Coprocessor;
import frc.lib.subsystems.drive.Drive;

public class ROSApriltag implements ROSPackage {
    private final NetworkTable apriltagTable;
    private final NetworkTableEntry apriltagIDs, apriltagTransforms, latency;
    private final AprilTagFieldLayout field;
    private final Drive drive;
    private final Transform3d cameraTransform;
    public ROSApriltag(Drive drive, Coprocessor processor, AprilTagFieldLayout field, Transform3d cameraTransform)
    {
        this.drive = drive;
        this.apriltagTable = processor.getROSTable("apriltag");
        this.field = field;
        this.apriltagIDs = apriltagTable.getEntry("ids");
        this.apriltagTransforms = apriltagTable.getEntry("transforms");
        this.cameraTransform = cameraTransform;
        this.latency = apriltagTable.getEntry("latency");
    }
    @Override
    public void update()
    {
        long[] tagIDs = apriltagIDs.getIntegerArray(new long[]{});
        Transform3d[] transforms = new Transform3d[tagIDs.length];
        double[] transformsRaw = apriltagTransforms.getDoubleArray(new double[]{});
        double latency = this.latency.getDouble(0);
        assert transformsRaw.length * 6 == tagIDs.length : "Invalid data sent by coprocessor";
        for (int i = 0; i < tagIDs.length; i++)
        {
            transforms[i] = new Transform3d(
                new Translation3d(
                    transformsRaw[i * 6],
                    transformsRaw[i * 6 + 1],
                    transformsRaw[i * 6 + 2]
                ),
                new Rotation3d(
                    transformsRaw[i * 6 + 3],
                    transformsRaw[i * 6 + 4],
                    transformsRaw[i * 6 + 5]
                )
            );
            var tagPose = field.getTagPose(i);
            if (tagPose.isPresent())
            {
                drive.addVisionEstimate(tagPose
                    .get()
                    .transformBy(transforms[i].inverse())
                    .transformBy(cameraTransform.inverse())
                    .toPose2d(), latency);
            }
        }
    }
}
