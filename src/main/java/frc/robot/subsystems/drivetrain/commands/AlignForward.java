package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;

public class AlignForward extends Command {
    private final Drivetrain m_drivetrain;
    private final PIDController yPID = new PIDController(0.5, 0.0, 0.00);
    
    // Variables to track last known target position
    private double lastKnownTargetDistance = 0.0;
    private boolean hasSeenTarget = false;
    
    // Tolerance and parameters for target tracking
    private static final double TARGET_DISTANCE_TOLERANCE = 0.1; // meters
    private static final double MAX_SEARCH_DISTANCE = 0.5; // maximum distance to continue moving

    public AlignForward(Drivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        addRequirements(drivetrain);
        
        // Configure PID controller
        yPID.setTolerance(TARGET_DISTANCE_TOLERANCE);
    }

    @Override
    public void initialize() {
        yPID.reset();
        hasSeenTarget = false;
        lastKnownTargetDistance = 0.0;
    }

    @Override
    public void execute() {
        try {
            // Try to get current target position
            var targetPose = LimelightLib.getTargetPose3d_CameraSpace("limelight-front");
            
            if (targetPose != null) {
                // Target is visible
                double targetOffsetY = targetPose.getZ();
                double yPidOutput = yPID.calculate(targetOffsetY, 0.32);
                
                m_drivetrain.drive(yPidOutput, 0, 0, false);
                
                // Update last known position
                lastKnownTargetDistance = targetOffsetY;
                hasSeenTarget = true;
            } else {
                // Target is not visible
                if (hasSeenTarget) {
                    // Continue moving towards last known position
                    double yPidOutput = yPID.calculate(lastKnownTargetDistance, 0.32);
                    
                    // Prevent continuous movement if far from last known position
                    if (Math.abs(lastKnownTargetDistance) <= MAX_SEARCH_DISTANCE) {
                        m_drivetrain.drive(yPidOutput, 0, 0, false);
                    } else {
                        // Stop if we've moved too far without seeing the target
                        m_drivetrain.drive(0, 0, 0, false);
                    }
                } else {
                    // No target has been seen yet, do nothing
                    m_drivetrain.drive(0, 0, 0, false);
                }
            }
        } catch (Exception e) {
            // Handle any unexpected errors
            m_drivetrain.drive(0, 0, 0, false);
            System.err.println("Error in AlignForward: " + e.getMessage());
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        // You might want to add more sophisticated ending conditions
        return false;
    }
}