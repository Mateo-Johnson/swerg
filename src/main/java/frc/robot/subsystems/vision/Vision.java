package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;

public class Vision extends SubsystemBase {
    private final Drivetrain m_drivetrain;
    private final String limelightName = "limelight-front";
    
    // Limelight mounting parameters (in meters and degrees)
    private static final double mountX = 0.0; // X offset from robot center (forward/back)
    private static final double mountY = 0.0; // Y offset from robot center (left/right)
    private static final double mountZ = 0.0; // Z height from ground
    private static final double mountPitch = 0.0; // Pitch angle (degrees)
    private static final double mountYaw = 0.0; // Yaw angle (degrees)
    private static final double mountRoll = 0.0; // Roll angle (degrees)
    
    // Vision pose rejection parameters
    private static final double maxDeviation = 2.0; // Maximum allowed deviation in meters between vision and odometry
    private boolean enableVisionUpdates = true; // Default to enabled
    private static final double minArea = 0.1; // Minimum area for a valid target
    private static final int minDetected = 1; // Minimum number of AprilTags needed
    
    /**
     * Creates a new Vision subsystem that automatically updates pose estimation
     * when AprilTags are detected.
     * 
     * @param drivetrain The drivetrain subsystem to update with vision measurements
     */
    public Vision(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        
        // Apply initial configuration to Limelight
        updateLimelightConfig();
    }
    
    @Override
    public void periodic() {
        // This method is called once per scheduler run
        // Automatically process vision data and update pose estimator if AprilTags are detected
        processVisionData();
    }
    
    /**
     * Process vision data from Limelight and update the pose estimator when AprilTags are detected
     */
    public void processVisionData() {
        // Skip processing if vision updates are disabled
        if (!enableVisionUpdates) {
            return;
        }
        
        // Check if Limelight has a valid target
        if (!LimelightLib.getTV(limelightName)) {
            return; // No valid target
        }
        
        // Get number of AprilTags visible
        int numTags = (int) NetworkTableInstance.getDefault()
            .getTable(limelightName)
            .getEntry("ntags")
            .getDouble(0);
            
        // Get target area to assess quality
        double targetArea = LimelightLib.getTA(limelightName);
        
        // Check minimum quality requirements
        if (numTags < minDetected || targetArea < minArea) {
            return; // Not enough AprilTags or target area too small
        }
        
        // Get the robot pose from Limelight
        Pose2d rawVisionPose = LimelightLib.getBotPose2d_wpiBlue(limelightName);
        
        // Transform to match our coordinate system (invert X and Y as in Drivetrain class)
        Pose2d visionPose = new Pose2d(
            -rawVisionPose.getX(),
            -rawVisionPose.getY(),
            rawVisionPose.getRotation()
        );
        
        // Calculate total latency (pipeline + capture) in seconds
        double latencySeconds = (LimelightLib.getLatency_Pipeline(limelightName) + 
                                LimelightLib.getLatency_Capture(limelightName)) / 1000.0;
        
        // Get current odometry pose for sanity check
        Pose2d currentPose = m_drivetrain.getPose();
        double poseDifference = calculatePoseDifference(currentPose, visionPose);
        
        // Only update if the vision measurement is reasonable
        if (poseDifference <= maxDeviation) {
            // Update the pose estimator with the vision measurement
            m_drivetrain.updateWithVision(visionPose, latencySeconds);
        }
    }
    
    /**
     * Calculate the Euclidean distance between two poses (ignoring rotation)
     */
    private double calculatePoseDifference(Pose2d pose1, Pose2d pose2) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();
        return Math.sqrt(dx*dx + dy*dy);
    }
    
    /**
     * Update the Limelight configuration with mounting parameters
     */
    private void updateLimelightConfig() {
        // Set the physical mounting parameters
        LimelightLib.setCameraPose_RobotSpace(
            limelightName,
            mountX, mountY, mountZ,
            mountRoll, mountPitch, mountYaw
        );
        
        // Set the pipeline (0 = default) - adjust if needed
        LimelightLib.setPipelineIndex(limelightName, 0);
    }
    
    /**
     * Set whether vision updates are enabled
     * @param enabled Whether to enable vision updates
     */
    public void setVisionEnabled(boolean enabled) {
        this.enableVisionUpdates = enabled;
    }
    
    /**
     * Force a reset of the pose estimator using vision data
     */
    public void forceVisionReset() {
        if (LimelightLib.getTV(limelightName)) {
            // Get number of AprilTags visible
            int numTags = (int) NetworkTableInstance.getDefault()
                .getTable(limelightName)
                .getEntry("ntags")
                .getDouble(0);
                
            // Only reset if we have AprilTags
            if (numTags >= minDetected) {
                Pose2d rawVisionPose = LimelightLib.getBotPose2d_wpiBlue(limelightName);
                Pose2d visionPose = new Pose2d(
                    -rawVisionPose.getX(),
                    -rawVisionPose.getY(),
                    rawVisionPose.getRotation()
                );
                
                // Reset odometry to vision pose
                m_drivetrain.resetOdometry(visionPose);
            }
        }
    }
}