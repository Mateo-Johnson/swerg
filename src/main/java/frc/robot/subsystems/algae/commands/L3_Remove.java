package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveToPoint;

/**
 * A command that runs three commands in sequence.
 * This demonstrates how to chain multiple commands together.
 */
public class L3_Remove extends SequentialCommandGroup {
  /**
   * Creates a new ThreeCommandSequence.
   *
   * @param m_elevator The elevator subsystem used by this command.
   * @param m_algae The coral subsystem used by this command.
   */
  public L3_Remove(Elevator m_elevator, Algae m_algae) {
    addCommands(
      new ParallelCommandGroup( // Move to target position, while updating the algae flipper over time
      new MoveToPoint(m_elevator, 15.5).withTimeout(0.3),
      new MoveOverTime(m_algae, 0.3, 0.5)
      ),
      new MoveToPosition(m_algae, 0.3).withTimeout(0.5) // Move the algae flipper to the target position (flip the algae)
    );
  }
}