package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveToPoint;

/**
 * A command that runs three commands in sequence.
 * This demonstrates how to chain multiple commands together.
 */
public class L2_Remove extends SequentialCommandGroup {
  /**
   * Creates a new ThreeCommandSequence.
   *
   * @param m_elevator The elevator subsystem used by this command.
   * @param m_algae The coral subsystem used by this command.
   */
  public L2_Remove(Elevator m_elevator, Algae m_algae) {
    addCommands(

    new MoveToPosition(m_algae, -1.5),
    new MoveToPoint(m_elevator, 13).withTimeout(1),
    new MoveToPosition(m_algae, 0) // Move the algae flipper to the target position (flip the algae)
    );
  }
}