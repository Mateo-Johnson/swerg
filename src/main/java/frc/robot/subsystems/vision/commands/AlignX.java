package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignX extends Command {
  private final double target;
  private final Drivetrain drivetrain;
  private final PIDController xPID = new PIDController(0.0, 0.0, 0.0);
  private final double currentMeasure;
  /** Creates a new AlignRotate. */
  public AlignX(double target, double currentMeasure, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    this.currentMeasure = currentMeasure;
    addRequirements(drivetrain);

    xPID.setTolerance(0.01); // 0.01 unit of the current measure, in this case it is meters (1 cm)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xTarget = target;
    double current = currentMeasure;
    double output = xPID.calculate(current, xTarget);
    output = Math.max(-1, Math.min(1, output));

    drivetrain.drive(output, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
