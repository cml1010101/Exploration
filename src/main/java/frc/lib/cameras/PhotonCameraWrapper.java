package frc.lib.cameras;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

public class PhotonCameraWrapper implements CameraWrapper {
    private final Transform3d transformToCenter;
    private final PhotonPoseEstimator visionEstimator;
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private final SimVisionSystem simulation;
    public PhotonCameraWrapper(String name, Transform3d transform, AprilTagFieldLayout layout, double fov, double ledRange,
        int cameraResolutionWidth, int cameraResolutionHeight, double minTargetArea)
    {
        camera = new PhotonCamera(name);
        transformToCenter = transform;
        visionEstimator = new PhotonPoseEstimator(layout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, camera, transform);
        updateLatestResult();
        if (RobotBase.isSimulation())
        {
            simulation = new SimVisionSystem(name, fov, transform, ledRange, cameraResolutionWidth, cameraResolutionHeight,
                minTargetArea);
            simulation.addVisionTargets(layout);
        }
        else
        {
            simulation = null;
        }
    }
    /**
     * Returns the Transform3d (X,Y,Z comp and Pitch, Roll, Yaw comp)
     * @return Transform 3d: Camera to Robot Center on floor
     */
    @Override
    public Transform3d getTransform()
    {
        return transformToCenter;
    }
    /**
     * Set the pipeline index
     * @param index index of the pipeline.
     */
    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }
    /**
     * Gets the index of the pipeline
     * @return index of the current pipeline
     */
    @Override
    public int getPipelineIndex() {
        return camera.getPipelineIndex();
    }
    /**
     * Updates the latest camera result
     */
    public void updateLatestResult() {
        latestResult = camera.getLatestResult();
    }
    /**
     * Gets whether a target exists
     * @return whether the camera sees a target
     */
    @Override
    public boolean hasTarget() {
        return latestResult.hasTargets();
    }
    /**
     * Gets yaw to target
     * @return target yaw
     */
    @Override
    public Rotation2d getTargetYaw() {
        return Rotation2d.fromDegrees(latestResult.getBestTarget().getYaw());
    }
    /**
     * Gets pitch to target
     * @return target pitch
     */
    @Override
    public Rotation2d getTargetPitch() {
        return Rotation2d.fromDegrees(latestResult.getBestTarget().getPitch());
    }
    /**
     * Gets target distance
     * @return target distance in meters
     */
    @Override
    public double getTargetDistance(double estimatedTargetHeight) {
        return PhotonUtils.calculateDistanceToTargetMeters(
            transformToCenter.getZ(),
            estimatedTargetHeight,
            transformToCenter.getRotation().getY(),
            getTargetPitch().getRadians()
        );
    }
    /**
     * Uses the apriltag ability to calculate the position of the robot
     * @param referencePose the current computed position by the encoders
     * @return the position of the robot estimated by the camera
     */
    public EstimatedRobotPose estimatePose(Pose2d referencePose)
    {
        if (RobotBase.isSimulation())
        {
            simulation.processFrame(referencePose);
        }
        visionEstimator.setReferencePose(referencePose);
        var results = visionEstimator.update();
        if (results.isPresent()) return results.get();
        else return null;
    }
}
