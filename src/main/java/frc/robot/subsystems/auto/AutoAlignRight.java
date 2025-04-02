package frc.robot.subsystems.auto;

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

public class AutoAlignRight extends Command {
  private final Drivetrain drivetrain;
  private final PIDController yPID = new PIDController(0.5, 0.0, 0.00);
  private final PIDController xPID = new PIDController(0.5, 0.0, 0.00);
  private final PIDController turnPID = new PIDController(0.032, 0, 0.0015);
  private final CommandXboxController prim = Constants.primary;
  private final String limelightName = "limelight-front";
  private double targetAngle;
  double turnOutput;
  
  // New variables for sequential alignment
  private static final double XY_DISTANCE_THRESHOLD = 0.25; // Threshold for when to start rotation (in meters)
  private boolean positionAligned = false; // Track if XY position is close enough
  
  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignLeft command that always aligns to the left of an AprilTag/target.
   * Now with sequential alignment - position first, then rotation.
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public AutoAlignRight(Drivetrain drivetrain) {
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
    xPID.reset();
    turnPID.reset();
    turnPID.enableContinuousInput(-180, 180);
    positionAligned = false; // Reset position alignment flag
    
    isAligning = true;
    SmartDashboard.putBoolean("Vision/AlignmentActive", true);
  }

  @Override
  public void execute() {
    // Always use left setpoint
    double currentSetpoint = AutoConstants.rightSetpoint;
    
    // Check if we have a valid target
    if (!LimelightLib.getTV(limelightName)) {
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
    double longitudinalOffset = targetPose.getY(); // Added to calculate XY distance
    
    // Calculate position error magnitude
    double xyErrorMagnitude = Math.sqrt(
      Math.pow(lateralOffset - currentSetpoint, 2) + 
      Math.pow(longitudinalOffset, 2)
    );
    
    // Update position aligned flag
    if (xyErrorMagnitude <= XY_DISTANCE_THRESHOLD) {
      positionAligned = true;
    }
    
    // Calculate PID outputs
    double lateralOutput = yPID.calculate(lateralOffset, currentSetpoint);
    double longitudinalOutput = xPID.calculate(longitudinalOffset, 0);
    
    // Clamp the outputs to valid ranges
    lateralOutput = MathUtil.clamp(lateralOutput, -0.5, 0.5);
    longitudinalOutput = MathUtil.clamp(longitudinalOutput, -0.5, 0.5);

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

    // Only perform rotational alignment if XY position is close enough
    if (positionAligned) {
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
      longitudinalOutput, // Use PID for forward/backward instead of joystick while aligning
      -lateralOutput,
      -turnOutput,
      false
    );

    // Log data for debugging
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/LongitudinalOffset", longitudinalOffset);
    SmartDashboard.putNumber("Vision/LateralOutput", lateralOutput);
    SmartDashboard.putNumber("Vision/CurrentSetpoint", currentSetpoint);
    SmartDashboard.putBoolean("Vision/TargetInView", true);
    SmartDashboard.putNumber("Vision/TargetID", LimelightLib.getFiducialID(limelightName));
    SmartDashboard.putBoolean("Vision/LateralAtSetpoint", yPID.atSetpoint());
    SmartDashboard.putNumber("Vision/XYErrorMagnitude", xyErrorMagnitude);
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
    
    // Check if we're at the setpoint
    return yPID.atSetpoint();
  }
}