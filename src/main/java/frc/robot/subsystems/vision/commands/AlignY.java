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
  
  // New variables for flick detection
  private boolean setpointLocked = false;
  private double lastXInput = 0;
  private static final double FLICK_THRESHOLD = 0.3;
  private static final int RESET_DELAY = 10; // Cycles before allowing new setpoint selection
  private int resetCounter = 0;

  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignY command that aligns to a specific AprilTag/target with multiple setpoints.
   * Uses a "flick" behavior for setpoint selection rather than continuous stick holding.
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
    currentSetpoint = centerSetpoint; // Start with center setpoint
    setpointLocked = false; // Start unlocked
    lastXInput = 0;
    resetCounter = 0;

    isAligning = true;
    
    SmartDashboard.putBoolean("Vision/AlignmentActive", true);
  }

  @Override
  public void execute() {
    // Get stick input
    double leftXInput = MathUtil.applyDeadband(prim.getLeftX(), OIConstants.kDriveDeadband);
    
    // Flick detection logic
    if (!setpointLocked) {
      // If not locked, check for flicks
      if (Math.abs(leftXInput) > FLICK_THRESHOLD) {
        // Flick detected
        if (leftXInput < -FLICK_THRESHOLD) {
          currentSetpoint = leftSetpoint;
        } else if (leftXInput > FLICK_THRESHOLD) {
          currentSetpoint = rightSetpoint;
        }
        setpointLocked = true;
      }
    } else {
      // If locked, check for return to center
      if (Math.abs(leftXInput) < OIConstants.kDriveDeadband) {
        resetCounter++;
        if (resetCounter >= RESET_DELAY) {
          // Allow for new setpoint selection after stick returns to center
          setpointLocked = false;
          resetCounter = 0;
        }
      } else {
        resetCounter = 0;
      }
      
      // Check for deliberate center selection (quick center flick)
      if (Math.abs(lastXInput) > FLICK_THRESHOLD && Math.abs(leftXInput) < OIConstants.kDriveDeadband) {
        currentSetpoint = centerSetpoint;
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
    SmartDashboard.putBoolean("Vision/SetpointLocked", setpointLocked);
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
    
    // Use PID controller to determine if we're aligned
    return yPID.atSetpoint();
  }
}