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

public class Align extends Command {
  private final Drivetrain drivetrain;
  private double currentSetpoint;
  private final PIDController yPID = new PIDController(0.5, 0.0, 0.00); 
  private final PIDController xPID = new PIDController(0.5, 0.0, 0.00); 
  private final PIDController turnPID = new PIDController(0.032, 0, 0.0015);
  private final CommandXboxController prim = Constants.primary;
  private final String limelightName = "limelight-front";
  private double targetAngle;
  double turnOutput;
  
  // Variables for flick detection
  private boolean initialSelectionMade = false;  // Track if initial selection has been made
  @SuppressWarnings("unused")
  private double lastXInput = 0;
  private static final double flickThreshold = 0.3;
  private static final double switchThreshold = 0.7;
  
  // New variables for sequential alignment
  private static final double XY_DISTANCE_THRESHOLD = 0.25; // Threshold for when to start rotation (in meters)
  private static final double LATERAL_ALIGNMENT_THRESHOLD = 0.05; // Threshold for lateral alignment (in meters)
  private boolean positionAligned = false; // Track if XY position is close enough
  private boolean lateralAligned = false; // Track if lateral position is close enough
  
  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignY command that aligns to the left or right of an AprilTag/target.
   * The robot starts in center position but can only align to left or right.
   * Allows direct switching between left and right with a strong flick.
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public Align(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Configure PID controllers
    yPID.setTolerance(0);
    xPID.setTolerance(0);
    turnPID.setTolerance(0);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controllers when command starts
    yPID.reset();
    turnPID.reset();
    xPID.reset();
    turnPID.enableContinuousInput(-180, 180);
    currentSetpoint = AutoConstants.centerSetpoint; // Start with center setpoint temporarily
    initialSelectionMade = false; // Start with no selection made
    lastXInput = 0;
    positionAligned = false; // Reset position alignment flag
    lateralAligned = false; // Reset lateral alignment flag

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
      if (Math.abs(leftXInput) > flickThreshold) {
        // First flick detected - set initial alignment direction
        if (leftXInput < -flickThreshold) {
          currentSetpoint = AutoConstants.leftSetpoint;
        } else if (leftXInput > flickThreshold) {
          currentSetpoint = AutoConstants.rightSetpoint;
        }
        initialSelectionMade = true;
      }
    } else {
      // Already aligned to a side, check for side switch
      // Strong flick in left direction
      if (leftXInput < -switchThreshold && currentSetpoint != AutoConstants.leftSetpoint) {
        currentSetpoint = AutoConstants.leftSetpoint;
        positionAligned = false; // Reset position alignment when changing sides
        lateralAligned = false; // Reset lateral alignment when changing sides
      } 
      // Strong flick in right direction
      else if (leftXInput > switchThreshold && currentSetpoint != AutoConstants.rightSetpoint) {
        currentSetpoint = AutoConstants.rightSetpoint;
        positionAligned = false; // Reset position alignment when changing sides
        lateralAligned = false; // Reset lateral alignment when changing sides
      }
    }
    
    // Store current input for next cycle
    lastXInput = leftXInput;
    
    // Check if we have a valid target
    if (!Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).contains((int)LimelightLib.getFiducialID(limelightName))) { // Checking if we are at the reef
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
    
    // Calculate PID outputs
    double lateralOutput = yPID.calculate(lateralOffset, currentSetpoint);
    
    // Check if we've achieved lateral alignment
    double lateralError = Math.abs(lateralOffset - currentSetpoint);
    if (lateralError <= LATERAL_ALIGNMENT_THRESHOLD) {
      lateralAligned = true;
    }
    
    // Only calculate longitudinal output if lateral alignment is achieved
    double longitudinalOutput = 0;
    if (initialSelectionMade && lateralAligned) {
      longitudinalOutput = xPID.calculate(longitudinalOffset, 0.3);
      longitudinalOutput = MathUtil.clamp(longitudinalOutput, -0.7, 0.7);
      
      // Calculate position error magnitude (only after lateral alignment)
      double xyErrorMagnitude = Math.sqrt(
        Math.pow(lateralOffset - currentSetpoint, 2) + 
        Math.pow(longitudinalOffset - 0.3, 2)
      );
      
      // Update position aligned flag
      positionAligned = (xyErrorMagnitude <= XY_DISTANCE_THRESHOLD);
      
      SmartDashboard.putNumber("Vision/XYErrorMagnitude", xyErrorMagnitude);
    } else {
      // If no lateral alignment yet, use joystick for forward/backward
      longitudinalOutput = MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband);
    }
    
    // Clamp the lateral output to valid range
    lateralOutput = MathUtil.clamp(lateralOutput, -0.7, 0.7);

    // Set target angle based on target ID
    switch ((int)LimelightLib.getFiducialID(limelightName)) {
      case 17: // 60°
      targetAngle = 60;
      break;
    case 8: // 60°
      targetAngle = 60;
      break;

    case 18: // 0°
      targetAngle = 0;
      break;
    case 7: // 0°
      targetAngle = 0;
      break;

    case 19: // -60°
      targetAngle = -60;
      break;
    case 6: // -60°
      targetAngle = -60;
      break;

    case 20: // -120°
      targetAngle = -120;
      break;
    case 11: // -120°
      targetAngle = -120;
      break;

    case 21: // 180°
      targetAngle = 180;
      break;
    case 10: // 180°
      targetAngle = 180;
      break;

    case 22: // 120°
      targetAngle = 120;
      break;
    case 9: // 120°
      targetAngle = 120;
      break;  
    }

    double angleError = targetAngle - drivetrain.getHeading();
    if (angleError > 180) {
      angleError -= 360;
    } else if (angleError < -180) {
      angleError += 360;
    }

    double currentAngleError = Math.abs(angleError);

    // Only perform rotational alignment if XY position is close enough and lateral alignment achieved
    if (positionAligned && lateralAligned) {
      turnOutput = turnPID.calculate(drivetrain.getHeading(), targetAngle);
      
      if (currentAngleError <= 5) { 
        // If we're under 5 degrees away from target use the joystick
        turnOutput = -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband);
      }
    } else {
      // Use joystick for rotation while positioning
      turnOutput = -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband);
    }
    
    // Apply both lateral and rotational corrections
    drivetrain.drive(
      MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
      -lateralOutput,
      -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband),
      false
    );

    // Log data for debugging
    SmartDashboard.putNumber("Vision/LongitudinalOffset", longitudinalOffset);
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/LateralError", lateralError);
    SmartDashboard.putNumber("Vision/LateralOutput", lateralOutput);
    SmartDashboard.putNumber("Vision/LongitudinalOutput", longitudinalOutput);
    SmartDashboard.putNumber("Vision/CurrentSetpoint", currentSetpoint);
    SmartDashboard.putBoolean("Vision/TargetInView", true);
    SmartDashboard.putNumber("Vision/TargetID", LimelightLib.getFiducialID(limelightName));
    SmartDashboard.putBoolean("Vision/LateralAtSetpoint", yPID.atSetpoint());
    SmartDashboard.putBoolean("Vision/InitialSelectionMade", initialSelectionMade);
    SmartDashboard.putBoolean("Vision/LateralAligned", lateralAligned);
    SmartDashboard.putNumber("Vision/JoystickX", leftXInput);
    SmartDashboard.putBoolean("Vision/PositionAligned", positionAligned);
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
    
    // Check if a side has been selected and we're at the setpoint
    return initialSelectionMade && yPID.atSetpoint();
  }
}