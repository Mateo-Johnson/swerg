package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.OIConstants;

public class AlignY extends Command {
  private final Drivetrain drivetrain;
  private final double centerSetpoint = 0;
  private final double leftSetpoint = 0.22;
  private final double rightSetpoint = -0.12;
  private double currentSetpoint;
  private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); //0.4
  private final CommandXboxController prim = Constants.primary;
  private final String limelightName = "limelight-front";
  
  // Variables for flick detection
  private boolean initialSelectionMade = false;  // Track if initial selection has been made
  private double lastXInput = 0;
  private static final double FLICK_THRESHOLD = 0.3;
  private static final double SWITCH_THRESHOLD = 0.7;  // Higher threshold for direct side switching

  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignY command that aligns to the left or right of an AprilTag/target.
   * The robot starts in center position but can only align to left or right.
   * Allows direct switching between left and right with a strong flick.
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignY(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Configure PID controllers
    yPID.setTolerance(0);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controllers when command starts
    yPID.reset();
    currentSetpoint = centerSetpoint; // Start with center setpoint temporarily
    initialSelectionMade = false; // Start with no selection made
    lastXInput = 0;

    isAligning = true;
    
    SmartDashboard.putBoolean("Vision/AlignmentActive", true);
  }

  @Override
  public void execute() {
    // Get stick input
    double leftXInput = MathUtil.applyDeadband(prim.getLeftX(), OIConstants.kDriveDeadband);
    
    // Selection logic
    if (!initialSelectionMade) {
      // Initial selection from center
      if (Math.abs(leftXInput) > FLICK_THRESHOLD) {
        // First flick detected - set initial alignment direction
        if (leftXInput < -FLICK_THRESHOLD) {
          currentSetpoint = leftSetpoint;
        } else if (leftXInput > FLICK_THRESHOLD) {
          currentSetpoint = rightSetpoint;
        }
        initialSelectionMade = true;
      }
    } else {
      // Already aligned to a side, check for side switch
      // Strong flick in left direction
      if (leftXInput < -SWITCH_THRESHOLD && currentSetpoint != leftSetpoint) {
        currentSetpoint = leftSetpoint;
      } 
      // Strong flick in right direction
      else if (leftXInput > SWITCH_THRESHOLD && currentSetpoint != rightSetpoint) {
        currentSetpoint = rightSetpoint;
      }
    }
    
    // Store current input for next cycle
    lastXInput = leftXInput;
    
    // Check if we have a valid target
    if (!LimelightLib.getTV(limelightName)) {
      // No valid target found, allow normal driving
      drivetrain.drive(
        MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
        leftXInput,
        -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
        true
      );
      SmartDashboard.putBoolean("Vision/TargetInView", false);
      return;
    }
    
    // Get the target pose in camera space - this is more direct for alignment
    Pose3d targetPose = LimelightLib.getTargetPose3d_CameraSpace(limelightName);
    
    if (targetPose == null) {
      drivetrain.drive(0, 0, 0, false);
      SmartDashboard.putBoolean("Vision/TargetInView", false);
      return;
    }
    
    // Extract the lateral offset (X-axis in camera space)
    double lateralOffset = targetPose.getX();
    
    // Calculate PID outputs
    double lateralOutput = yPID.calculate(lateralOffset, currentSetpoint);
    
    // Clamp the outputs to valid ranges
    lateralOutput = MathUtil.clamp(lateralOutput, -0.5, 0.5);
    
    // Apply both lateral and rotational corrections
    drivetrain.drive(
      MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
      -lateralOutput,
      -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
      false
    );

    // Log data for debugging
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/LateralOutput", lateralOutput);
    SmartDashboard.putNumber("Vision/CurrentSetpoint", currentSetpoint);
    SmartDashboard.putBoolean("Vision/TargetInView", true);
    SmartDashboard.putNumber("Vision/TargetID", LimelightLib.getFiducialID(limelightName));
    SmartDashboard.putBoolean("Vision/LateralAtSetpoint", yPID.atSetpoint());
    SmartDashboard.putBoolean("Vision/InitialSelectionMade", initialSelectionMade);
    SmartDashboard.putNumber("Vision/JoystickX", leftXInput);
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
    // Check if we have valid target data
    if (!LimelightLib.getTV(limelightName)) {
      return false;
    }
    
    // Check if a side has been selected and we're at the setpoint
    return initialSelectionMade && yPID.atSetpoint();
  }
}