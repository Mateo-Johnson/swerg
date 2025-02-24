package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.PIDConstants;

public class AlignRotate extends Command {
  private final double angle;
  private final double currentMeasure;
  private final Drivetrain drivetrain;
  private final PIDController rotatePID = PIDConstants.rotateController;

  /**
   * Creates a new AlignRotate command.
   *
   * @param angle The target angle to align to the robot to (robot relative)
   * @param tolerance The tolerance on the current measure in this case it is generally degrees (1 = 1°)
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignRotate(double angle, double tolerance, Drivetrain drivetrain) {
    this(angle, tolerance, drivetrain.getHeading(), drivetrain);
  }

  /**
   * Creates a new AlignRotate command.
   *
   * @param angle The target angle to align to the robot to (robot relative)
   * @param currentMeasure The current measure of the angle.
   * @param tolerance The tolerance on the current measure in this case it is generally degrees (1 = 1°)
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignRotate(double angle, double tolerance, double currentMeasure, Drivetrain drivetrain) {
    this.angle = angle;
    this.currentMeasure = currentMeasure;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    rotatePID.setTolerance(tolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = angle;
    double currentAngle = currentMeasure; // Use the passed currentMeasure

    double output = rotatePID.calculate(currentAngle, targetAngle);
    output = Math.max(-1, Math.min(1, output));

    drivetrain.drive(0, 0, output, false);
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