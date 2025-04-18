package frc.robot.subsystems.drivetrain.commands;

import java.util.Arrays;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.LimelightLib;

/**
 * Command to align the robot to the left setpoint
 */
public class AlignLeft extends Command {
  private final Drivetrain drivetrain;
  private double targetSetpoint;
  private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 
  private final PIDController xPID = new PIDController(0.5, 0.0, 0.00); 
  private final CommandXboxController prim = Constants.primary;
  private final String limelightName = "limelight-front";
  
  // Alignment state tracking
  private static final double SIDE_ALIGNMENT_THRESHOLD = 0.05; // Threshold for side-to-side alignment (in meters)
  
  // Alignment state machine
  private enum AlignmentState {
    SIDE_ALIGNMENT, // Moving to the selected side
    LATERAL_ALIGNMENT // Moving forward/backward to target position
  }
  
  private AlignmentState currentState = AlignmentState.SIDE_ALIGNMENT;
  
  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignLeft command that aligns to the left setpoint
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignLeft(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Configure PID controllers
    yPID.setTolerance(0);
    xPID.setTolerance(0);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controllers when command starts
    yPID.reset();
    xPID.reset();
    currentState = AlignmentState.SIDE_ALIGNMENT;

    isAligning = true;
    
    SmartDashboard.putBoolean("Vision/AlignmentActive", true);
    SmartDashboard.putString("Vision/AlignmentState", currentState.toString());
  }

  @Override
  public void execute() {
    
    // Determine if controls should be inverted based on target ID
    boolean invertControls = Arrays.asList(20, 21, 22, 9, 10, 11).contains((int)LimelightLib.getFiducialID(limelightName));
    
    // If controls are inverted, change the sign of the leftXInput
    if (invertControls) {
      targetSetpoint = AutoConstants.rightSetpoint;
    } else {
      targetSetpoint = AutoConstants.leftSetpoint;
    }
    
    // Check if we have a valid target
    if (!Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).contains((int)LimelightLib.getFiducialID(limelightName))) { // Checking if we are at the reef
      // No valid target found, allow normal driving
      drivetrain.drive(
        MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
        MathUtil.applyDeadband(prim.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
        true
      );
      SmartDashboard.putBoolean("Vision/TargetInView", false);
      return;
    }
    
    // Get the target pose in camera space
    Pose3d targetPose = LimelightLib.getTargetPose3d_CameraSpace(limelightName);
    
    if (targetPose == null) {
      drivetrain.drive(0, 0, 0, false);
      SmartDashboard.putBoolean("Vision/TargetInView", false);
      return;
    }
    
    // Extract the lateral offset (X-axis in camera space)
    double lateralOffset = targetPose.getX();
    double longitudinalOffset = targetPose.getZ();
    
    // State machine for alignment process
    switch (currentState) {
      case SIDE_ALIGNMENT:
        // Calculate PID output for side-to-side movement
        double lateralError = lateralOffset - targetSetpoint;
        double lateralOutput = yPID.calculate(lateralOffset, targetSetpoint);
        lateralOutput = MathUtil.clamp(lateralOutput, -0.7, 0.7);
        
        // Check if we've achieved side-to-side alignment
        if (Math.abs(lateralError) <= SIDE_ALIGNMENT_THRESHOLD) {
          currentState = AlignmentState.LATERAL_ALIGNMENT;
        }
        
        // Apply side-to-side movement while using joystick for forward/backward and rotation
        drivetrain.drive(
          MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
          -lateralOutput,
          -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
          false
        );
        break;
        
      case LATERAL_ALIGNMENT:
        // Now focus on lateral (forward/backward) alignment
        double longitudinalOutput = xPID.calculate(longitudinalOffset, 0.21);
        longitudinalOutput = MathUtil.clamp(longitudinalOutput, -0.7, 0.7);
        
        // Side-to-side fine-tuning (with reduced gain)
        lateralError = lateralOffset - targetSetpoint;
        lateralOutput = yPID.calculate(lateralOffset, targetSetpoint);
        lateralOutput = MathUtil.clamp(lateralOutput, -0.5, 0.5); // Reduced maximum speed
        
        // Apply both lateral and longitudinal corrections, with manual rotation control
        drivetrain.drive(
          longitudinalOutput,
          -lateralOutput,
          -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
          false
        );
        break;
    }

    // Log data for debugging
    SmartDashboard.putString("Vision/AlignmentState", currentState.toString());
    SmartDashboard.putNumber("Vision/LongitudinalOffset", longitudinalOffset);
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/CurrentSetpoint", targetSetpoint);
    SmartDashboard.putBoolean("Vision/TargetInView", true);
    SmartDashboard.putNumber("Vision/TargetID", LimelightLib.getFiducialID(limelightName));
    SmartDashboard.putBoolean("Vision/LateralAtSetpoint", yPID.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    isAligning = false;
    drivetrain.drive(0, 0, 0, false);
    SmartDashboard.putBoolean("Vision/TargetInView", false);
    SmartDashboard.putBoolean("Vision/AlignmentActive", false);
  }

  @Override
  public boolean isFinished() {
    if (prim.povDown().getAsBoolean() || prim.povUp().getAsBoolean() || prim.povRight().getAsBoolean() || prim.povLeft().getAsBoolean()) {  
      return true;
    }

    // Check if we have valid target data
    if (!LimelightLib.getTV(limelightName)) {
      return false;
    }
    
    // Command never finishes on its own - must use D-pad to exit
    return false;
  }
}