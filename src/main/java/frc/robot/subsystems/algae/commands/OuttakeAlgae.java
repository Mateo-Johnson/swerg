package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

public class OuttakeAlgae extends Command {
  private final Algae algae;
  /** Creates a new OuttakeAlgae. */
  public OuttakeAlgae(Algae m_algae) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algae = m_algae;
    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.outtake();
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
