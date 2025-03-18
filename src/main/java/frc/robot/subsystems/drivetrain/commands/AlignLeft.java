package frc.robot.subsystems.drivetrain.commands;

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

public class AlignLeft extends Command {
  private final Drivetrain drivetrain;
  private final PIDController yPID = new PIDController(0.5, 0.0, 0.00);
  private final PIDController turnPID = new PIDController(0.032, 0, 0.0015);
  private final CommandXboxController prim = Constants.primary;
  private final String limelightName = "limelight-front";
  private double targetAngle;
  double turnOutput;
  
  public static boolean isAligning = false;
  
  /**
   * Creates a new AlignLeft command that always aligns to the left of an AprilTag/target.
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignLeft(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Configure PID controllers
    yPID.setTolerance(0);
    turnPID.setTolerance(0);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controllers when command starts
    yPID.reset();
    turnPID.reset();
    turnPID.enableContinuousInput(-180, 180);
    
    isAligning = true;
    SmartDashboard.putBoolean("Vision/AlignmentActive", true);
  }

  @Override
  public void execute() {
    // Always use left setpoint
    double currentSetpoint = AutoConstants.leftSetpoint;
    
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
    
    // Calculate PID outputs
    double lateralOutput = yPID.calculate(lateralOffset, currentSetpoint);
    
    // Clamp the outputs to valid ranges
    lateralOutput = MathUtil.clamp(lateralOutput, -0.5, 0.5);

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

    turnOutput = turnPID.calculate(drivetrain.getHeading(), targetAngle);

    if (currentAngleError <= 5) { 
      // If we're under 5 degrees away from target use the joystick
      turnOutput = -MathUtil.applyDeadband(prim.getRightX(), OIConstants.kDriveDeadband);
    }
    
    // Apply both lateral and rotational corrections
    drivetrain.drive(
      MathUtil.applyDeadband(prim.getLeftY(), OIConstants.kDriveDeadband),
      -lateralOutput,
      -turnOutput,
      false
    );

    // Log data for debugging
    SmartDashboard.putNumber("Vision/LateralOffset", lateralOffset);
    SmartDashboard.putNumber("Vision/LateralOutput", lateralOutput);
    SmartDashboard.putNumber("Vision/CurrentSetpoint", currentSetpoint);
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
    
    // Check if we're at the setpoint
    return yPID.atSetpoint();
  }
}