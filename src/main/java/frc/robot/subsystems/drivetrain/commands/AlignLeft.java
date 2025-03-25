package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.LimelightLib;

public class AlignLeft extends Command {
    private final Drivetrain m_drivetrain;
    private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 
    
    private static final double TARGET_TOLERANCE = 0; // Acceptable error for targeting

    public AlignLeft(Drivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
        
        // Add requirements to ensure command interacts with the swerve drive subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controller when the command starts
        yPID.reset();
    }

    @Override
    public void execute() {
        // Get the horizontal offset from Limelight (target offset in degrees)
        double targetOffsetX = LimelightLib.getTargetPose3d_CameraSpace("limelight-front").getX();
        double targetOffsetY = LimelightLib.getTargetPose3d_CameraSpace("limelight-front").getY();

        // Calculate the output from the PID controller to correct the heading towards the target
        double xPidOutput = yPID.calculate(targetOffsetX, AutoConstants.leftSetpoint);
        double yPidOutput = yPID.calculate(targetOffsetY, 0.3);

        // Set the swerve drive to drive towards the target
        m_drivetrain.drive(yPidOutput, xPidOutput, 0, false);
    }

    @Override
    public boolean isFinished() {
        // Finish when the PID controller is within an acceptable tolerance
        return Math.abs(yPID.getPositionError()) < TARGET_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve drive when the command ends
        m_drivetrain.drive(0, 0, 0, false);
    }
}
