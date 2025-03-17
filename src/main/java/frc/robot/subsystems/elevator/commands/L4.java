package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
import frc.robot.subsystems.elevator.Elevator;

/**
 * A command that runs three commands in sequence.
 * This demonstrates how to chain multiple commands together.
 */
public class L4 extends SequentialCommandGroup {
  /**
   * Creates a new ThreeCommandSequence.
   *
   * @param m_elevator The elevator subsystem used by this command.
   * @param m_coral The coral subsystem used by this command.
   */
  public L4(Elevator m_elevator, Coral m_coral) {
    addCommands(
      new MoveToPoint(m_elevator, 42.8).withTimeout(0.8),
      new Intake(m_coral, 0.7).withTimeout(0.12),
      new WaitCommand(0.3),
      new MoveManual(m_elevator, 0.1).withTimeout(0.21),
      // new WaitCommand(0.2),
      new Intake(m_coral, 0.4).withTimeout(0.5)
    );
  }
}