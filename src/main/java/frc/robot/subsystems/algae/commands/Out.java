package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

public class Out extends Command {
  private final Algae algae;

  /** Creates a new Out. */
  public Out(Algae algae) {  // Specify the type 'Algae' here
    this.algae = algae;  // Initialize the algae field with the passed parameter
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.pivotAlgae(0.34); //0.28
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
