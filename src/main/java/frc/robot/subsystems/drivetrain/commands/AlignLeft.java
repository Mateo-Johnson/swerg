package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.LimelightLib;

public class AlignLeft extends Command {
    private final Drivetrain m_drivetrain;
    private final PIDController xPID = new PIDController(0.5, 0.0, 0.00); 
    private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 

    private static final double TARGET_TOLERANCE = 0.05; // Acceptable error for targeting
    private boolean xAligned = false;

    public AlignLeft(Drivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        // Add requirements to ensure command interacts with the swerve drive subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controllers when the command starts
        xPID.reset();
        yPID.reset();
        xAligned = false;
    }

    @Override
    public void execute() {
        // Get the horizontal offset from Limelight (target offset in degrees)
        double targetOffsetX = LimelightLib.getTargetPose3d_CameraSpace("limelight-front").getX();
        double targetOffsetY = LimelightLib.getTargetPose3d_CameraSpace("limelight-front").getZ();

        SmartDashboard.putNumber("x", targetOffsetX);
        SmartDashboard.putNumber("y", targetOffsetY);

        double xPidOutput = xPID.calculate(targetOffsetX, 0.22);
        m_drivetrain.drive(0, -xPidOutput, 0, false);
    }

    @Override
    public boolean isFinished() {
        // Finish when both X and Y are aligned
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the swerve drive when the command ends
        m_drivetrain.drive(0, 0, 0, false);
    }
}