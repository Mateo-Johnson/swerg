package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.PIDConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignY extends Command {
  private final double target;
  private final Drivetrain drivetrain;
  private final PIDController yPID = PIDConstants.yPID;
  private final double currentMeasure;

  /**
   * Creates a new AlignY command.
   *
   * @param target The target y-position to align to
   * @param currentMeasure The current measurement value to use for alignment
   * @param tolerance The tolerance on the current measure in this case it is generally meters (0.01 = 1 cm) 
   * @param drivetrain The drivetrain subsystem to control
   */
  public AlignY(double target, double currentMeasure, double tolerance, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    this.currentMeasure = currentMeasure;
    addRequirements(drivetrain);

    yPID.setTolerance(tolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yTarget = target;
    double current = currentMeasure;
    double output = yPID.calculate(current, yTarget);
    output = Math.max(-1, Math.min(1, output));

    drivetrain.drive(0, output, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yPID.atSetpoint();
  }
}
