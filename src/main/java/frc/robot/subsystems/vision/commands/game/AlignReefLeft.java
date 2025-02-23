package frc.robot.subsystems.vision.commands.game;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.commands.general.AlignX;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReefLeft extends Command {
  /** Creates a new AlignReefLeft. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;

  public AlignReefLeft(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drivetrain;
    this.m_vision = vision;
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tX = m_vision.getTX();
    double setpoint = -5;
    
    new AlignX(setpoint, tX, 0.1, m_drivetrain).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
