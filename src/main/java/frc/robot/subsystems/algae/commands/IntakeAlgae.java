package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

public class IntakeAlgae extends Command {
  private final Algae algae;
  
  /**
   * Creates a new IntakeUntilStored command.
   * This command will run the intake until a game piece is detected,
   * then continue running in store mode until the command is interrupted.
   * 
   * @param m_algae The algae subsystem to use
   */
  public IntakeAlgae(Algae m_algae) {
    this.algae = m_algae;
    
    // This command requires the algae subsystem
    addRequirements(m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset detection state
    algae.resetGamePieceDetection();
    
    // Start intaking
    algae.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if we've detected a game piece
    if (algae.hasGamePiece()) {
      // Switch to store mode (slow intake)
      algae.store();
    } else {
      // Continue intaking at full speed
      algae.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motors when the command ends
    algae.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command runs until interrupted (button released)
    return false;
  }
}