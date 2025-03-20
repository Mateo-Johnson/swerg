package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.coral.commands.Intake;
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
   * @param m_coral The coral subsystem used by this command.
   */
  public L3_Remove(Elevator m_elevator, Coral m_coral) {
    addCommands(
      new MoveToPoint(m_elevator, 42.8).withTimeout(0.8)
    );
  }
}