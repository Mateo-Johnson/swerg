package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.LimelightResults;
import frc.robot.utils.LimelightLib.PoseEstimate;

public class Vision extends SubsystemBase {
    private final String limelightName;
    
    private static final double CAMERA_HEIGHT_METERS = 0.5; // Adjust based on robot
    private static final double CAMERA_FORWARD_METERS = 0.3; // Adjust based on robot
    private static final double CAMERA_SIDESWAY_METERS = 0.0; // Adjust based on robot
    private static final double CAMERA_PITCH_DEGREES = 30.0; // Adjust based on robot
    private static final double CAMERA_ROLL_DEGREES = 0.0; // Adjust based on robot
    private static final double CAMERA_YAW_DEGREES = 0.0; // Adjust based on robot

    /**
     * Creates a new Vision Subsystem with a custom Limelight name
     * @param limelightName The name of the Limelight camera to use
     */
    public Vision(String limelightName) {
        this.limelightName = limelightName;
        
        // Configure camera pose relative to robot
        LimelightLib.setCameraPose_RobotSpace(
            limelightName,
            CAMERA_FORWARD_METERS,
            CAMERA_SIDESWAY_METERS,
            CAMERA_HEIGHT_METERS,
            CAMERA_ROLL_DEGREES,
            CAMERA_PITCH_DEGREES,
            CAMERA_YAW_DEGREES
        );

        // Set LED mode to pipeline control
        LimelightLib.setLEDMode_PipelineControl(limelightName);
    }

    @Override
    public void periodic() {
        updateVisionData();
    }

    private void updateVisionData() {
        // Get basic targeting data
        double tx = LimelightLib.getTX(limelightName);
        double ty = LimelightLib.getTY(limelightName);
        double ta = LimelightLib.getTA(limelightName);
        boolean hasTarget = LimelightLib.getTV(limelightName);

        // Get detailed results
        LimelightResults results = LimelightLib.getLatestResults(limelightName);

        // Get pose estimate (for localization)
        PoseEstimate poseEstimate = LimelightLib.getBotPoseEstimate_wpiBlue(limelightName);
    }


    private double calculateDistance(double targetArea) {
        // Simple distance calculation based on target area
        // Adjust these constants based on your target size and camera settings
        return (27.0 / Math.sqrt(targetArea));
    }

    // Public methods for other subsystems to use
    public boolean hasTarget() {
        return LimelightLib.getTV(limelightName);
    }

    public double getTargetOffsetAngle() {
        return LimelightLib.getTX(limelightName);
    }

    public double getTargetDistance() {
        double area = LimelightLib.getTA(limelightName);
        return calculateDistance(area);
    }

    public Pose2d getRobotPose() {
        PoseEstimate poseEstimate = LimelightLib.getBotPoseEstimate_wpiBlue(limelightName);
        return poseEstimate != null ? poseEstimate.pose : new Pose2d();
    }

    public void setPipeline(int pipelineIndex) {
        LimelightLib.setPipelineIndex(limelightName, pipelineIndex);
    }

    public void setLEDMode(int mode) {
        switch (mode) {
            case 0 -> LimelightLib.setLEDMode_PipelineControl(limelightName);
            case 1 -> LimelightLib.setLEDMode_ForceOff(limelightName);
            case 2 -> LimelightLib.setLEDMode_ForceBlink(limelightName);
            case 3 -> LimelightLib.setLEDMode_ForceOn(limelightName);
        }
    }

public Pose3d getCurrentPosition() {
    // Get the latest pose estimate
    PoseEstimate poseEstimate = LimelightLib.getBotPoseEstimate_wpiBlue(limelightName);
    
    // Return null if no valid position is available
    if (poseEstimate == null || poseEstimate.tagCount < 1) {
        return null;
    }
    
    // Convert the pose estimate to Pose3d
    return new Pose3d(
        poseEstimate.pose.getX(),
        poseEstimate.pose.getY(),
        CAMERA_HEIGHT_METERS,
        new Rotation3d(poseEstimate.pose.getRotation())  // Direct conversion from Rotation2d to Rotation3d
    );
}
}