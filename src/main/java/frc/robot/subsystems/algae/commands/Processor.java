package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveToPoint;

/**
 * A command that runs three commands in sequence.
 * This demonstrates how to chain multiple commands together.
 */
public class Processor extends SequentialCommandGroup {
  /**
   * Creates a new ThreeCommandSequence.
   *
   * @param m_elevator The elevator subsystem used by this command.
   * @param m_algae The coral subsystem used by this command.
   */
  public Processor(Elevator m_elevator, Algae m_algae) {
    addCommands(
    new MoveToPoint(m_elevator, 0).withTimeout(1),
    new MoveToPosition(m_algae, -2) // Move the algae flipper to the target position (flip the algae)
    );
  }
}