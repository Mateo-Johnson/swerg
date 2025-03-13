package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AlignRotate extends Command {
  private final double targetAngle;
  private final Drivetrain drivetrain;
  private final PIDController rotatePID;

  /**
   * Creates a new AlignRotate command.
   *
   * @param angle The target angle to align to (in degrees)
   * @param tolerance The allowed error in degrees
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignRotate(double angle, Drivetrain drivetrain) {
    this.targetAngle = angle;
    this.drivetrain = drivetrain;
    this.rotatePID = new PIDController(0.032, 0, 0.0015);
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = drivetrain.getHeading();
    
    // Calculate PID output
    double output = rotatePID.calculate(currentAngle, targetAngle);
    
    // Limit the output to valid range
    output = MathUtil.clamp(output, -1, 1);
    
    // Drive the robot
    drivetrain.drive(0, 0, output, false);
    
    // Show debug values
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Rotation Output", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}