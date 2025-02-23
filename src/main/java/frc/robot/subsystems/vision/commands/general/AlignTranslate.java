package frc.robot.subsystems.vision.commands.general;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTranslate extends Command {
  private final Pose2d target;
  private final Drivetrain drivetrain;
  private final PIDController translatePID = new PIDController(0.0, 0.0, 0.0);
  /** Creates a new AlignRotate. */
  public AlignTranslate(Pose2d target, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    translatePID.setTolerance(0.01); // 0.01 unit of the current measure, in this case it is meters (1 cm)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
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
