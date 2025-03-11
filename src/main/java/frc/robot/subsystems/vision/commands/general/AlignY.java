package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.LimelightLib;

public class AlignY extends Command {
  private final Drivetrain drivetrain;
  private final PIDController yPID = new PIDController(0.015, 0.0001, 0);
  private final String limelightName = "limelight-front";
  
  // Target ID to track
  private final int targetID;
  
  /**
   * Creates a new AlignY command that aligns to a specific AprilTag/target.
   *
   * @param targetID The ID of the target to align with
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignY(int targetID, Drivetrain drivetrain) {
    this.targetID = targetID;
    this.drivetrain = drivetrain;
    
    // Configure PID controller
    yPID.setTolerance(0.02); // Set tolerance in meters
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controller when command starts
    yPID.reset();
  }

  @Override
  public void execute() {
    // Get the target pose
    double[] targetPose = LimelightLib.getTargetPose_RobotSpace("limelight-front");
    
    if (targetPose.length < 6) {
      // No valid target found, stop alignment
      drivetrain.drive(0, 0, 0, false);
      return;
    }
    
    // Extract the lateral offset (Y-axis in robot space)
    // targetPose format is [x, y, z, roll, pitch, yaw]
    double lateralOffset = targetPose[1];
    
    // Our goal is to make this lateral offset zero
    double output = yPID.calculate(lateralOffset, 0);
    
    // Clamp the output to valid range
    output = MathUtil.clamp(output, -0.5, 0.5);
    
    // Apply the steering correction while maintaining forward/backward control from driver
    drivetrain.drive(
      MathUtil.applyDeadband(Constants.primary.getLeftY(), OIConstants.kDriveDeadband), 
      -output, // Negate because positive output should turn robot right to align left
      0, 
      false);

    // Log data for debugging
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/AlignOutput", output);
    SmartDashboard.putBoolean("Vision/TargetInView", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
    SmartDashboard.putBoolean("Vision/TargetInView", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if we have valid target data
    double[] targetPose = LimelightLib.getTargetPose_RobotSpace("limelight-front");
    if (targetPose.length < 6) {
      return false;
    }
    
    // Use the PID controller to determine if we're at the setpoint
    return yPID.atSetpoint();
  }
}