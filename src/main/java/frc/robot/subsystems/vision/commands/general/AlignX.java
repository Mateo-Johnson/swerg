package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.PIDConstants;

/**
 * A command that aligns the robot along the X-axis to a target position using PID control.
 * This command handles x-axis translation only, maintaining the current y position and rotation.
 * The command uses a PID controller to compute the necessary drive output for x-axis movement.
 * 
 * <p>The command will continuously attempt to maintain the target x-position until interrupted
 * or explicitly ended by another command. The drive output is clamped between -1 and 1
 * to ensure safe operation.
 *
 * @see Command
 * @see Drivetrain
 * @see PIDController
 */
public class AlignX extends Command {
  private final double target;
  private final Drivetrain drivetrain;
  private final PIDController xPID = PIDConstants.xPID;
  private final double currentMeasure;

  /**
   * Creates a new AlignX command.
   *
   * @param target The target x-position to align to
   * @param currentMeasure The current measurement value to use for alignment
   * @param tolerance The tolerance on the current measure in this case it is generally meters (0.01 = 1 cm) 
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignX(double target, double currentMeasure, double tolerance, Drivetrain drivetrain) {
    this.target = target;
    this.drivetrain = drivetrain;
    this.currentMeasure = currentMeasure;
    addRequirements(drivetrain);

    xPID.setTolerance(tolerance);
  }

  /**
   * Called when the command is initially scheduled.
   * This method is called once when the command is scheduled.
   */
  @Override
  public void initialize() {}

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * This method:
   * 1. Gets the current and target x-positions
   * 2. Calculates PID output for x-axis movement
   * 3. Clamps output between -1 and 1
   * 4. Applies the calculated output to the drivetrain
   */
  @Override
  public void execute() {
    double xTarget = target;
    double current = currentMeasure;
    double output = xPID.calculate(current, xTarget);
    output = Math.max(-1, Math.min(1, output));

    drivetrain.drive(output, 0, 0, false);
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the drivetrain to ensure the robot doesn't continue moving.
   *
   * @param interrupted Whether the command was interrupted (true) or completed normally (false)
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  /**
   * Returns whether the command should end.
   * Currently always returns false, meaning the command will run indefinitely
   * until interrupted.
   *
   * @return false, indicating the command should continue running
   */
  @Override
  public boolean isFinished() {
    return xPID.atSetpoint();
  }
}