package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class MoveDistance extends Command {
  /** Creates a new TranslateDistance command. */
  private final double targetX; // Target X coordinate
  private final double targetY; // Target Y coordinate
  private final Drivetrain drivetrain;

  // PID Constants - these will need tuning
  double kP = 0.1;  // Proportional gain
  double kI = 0.01; // Integral gain
  double kD = 0.001; // Derivative gain
  
  // Create separate PID controllers for x and y translation
  PIDController xTranslatePID = new PIDController(kP, kI, kD);
  PIDController yTranslatePID = new PIDController(kP, kI, kD);

  public MoveDistance(Drivetrain drivetrain, double targetX, double targetY) {
    this.drivetrain = drivetrain;
    this.targetX = targetX;
    this.targetY = targetY;
    
    // Add subsystem requirements
    addRequirements(drivetrain);
    
    // Configure PID controller tolerances
    xTranslatePID.setTolerance(0.02); // 2 cm tolerance
    yTranslatePID.setTolerance(0.02); // 2 cm tolerance
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID controllers
    xTranslatePID.reset();
    yTranslatePID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current pose from drivetrain
    Pose2d currentPose = drivetrain.getPose();
    
    // Calculate PID outputs for x and y translation
    double xOutput = xTranslatePID.calculate(currentPose.getX(), targetX);
    double yOutput = yTranslatePID.calculate(currentPose.getY(), targetY);
    
    // Drive the robot using calculated PID outputs
    drivetrain.drive(xOutput, yOutput, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when both X and Y PID controllers are on target
    return xTranslatePID.atSetpoint() && yTranslatePID.atSetpoint();
  }
}