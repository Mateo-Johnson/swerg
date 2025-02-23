package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.PIDConstants;

/**
 * A command that aligns the robot's position to a target position using PID control.
 * This command handles x and y translation to reach the target pose, without considering rotation.
 * The command uses separate PID controllers for x and y axes to compute the necessary drive outputs.
 * 
 * <p>The command will continuously attempt to maintain the target position until interrupted
 * or explicitly ended by another command. The drive outputs are clamped between -1 and 1
 * to ensure safe operation.
 *
 * @see Command
 * @see Drivetrain
 * @see PIDController
 */
public class AlignTranslate extends Command {
  private final Pose2d target;
  private final Drivetrain drivetrain;
  private final PIDController translatePID = PIDConstants.translateController;

  /**
   * Creates a new AlignTranslate command.
   *
   * @param target The target pose to align to (only x and y coordinates are used)
   * @param tolerance The tolerance on the current measure in this case it is generally meters (0.01 = 1 cm)
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignTranslate(Pose2d target, double tolerance, Drivetrain drivetrain) {
    this.target = target;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    translatePID.setTolerance(tolerance);
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
   * 1. Gets the current and target positions
   * 2. Calculates PID outputs for x and y axes
   * 3. Clamps outputs between -1 and 1
   * 4. Applies the calculated outputs to the drivetrain
   */
  @Override
  public void execute() {
    double xTarget = target.getX();
    double yTarget = target.getY();
    double xCurrent = drivetrain.getPose().getX();
    double yCurrent = drivetrain.getPose().getY();

    double xOutput = translatePID.calculate(xCurrent, xTarget);
    xOutput = Math.max(-1, Math.min(1, xOutput));

    double yOutput = translatePID.calculate(yCurrent, yTarget);
    yOutput = Math.max(-1, Math.min(1, yOutput));

    drivetrain.drive(xOutput, yOutput, 0, false);
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
    return translatePID.atSetpoint();
  }
}