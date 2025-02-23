package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignRotate extends Command {
  private final double angle;
  private final Drivetrain drivetrain;
  private final PIDController rotatePID = new PIDController(0.0, 0.0, 0.0);
  /** Creates a new AlignRotate. */
  public AlignRotate(double angle, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    rotatePID.setTolerance(1); // 1 unit of the current measure, in this case it is degrees (1°)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = angle;
    double currentAngle = drivetrain.getHeading();

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
    return false;
  }
}
