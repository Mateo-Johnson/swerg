package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/**
 * A command that allows manual control of the elevator.
 * This command runs as long as it is held.
 */
public class ManualMove extends Command {
  private final Elevator elevator;
  private final double speed;

  /**
   * Creates a new ManualMoveCommand.
   * 
   * @param elevator The Elevator subsystem to be controlled.
   * @param speed The speed at which to move the elevator (positive for upward, negative for downward).
   */
  public ManualMove(Elevator elevator, double speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
    setName("ManualMove");
  }

  @Override
  public void initialize() {
    elevator.setManualControl(true); // Enable manual control
  }

  @Override
  public void execute() {
    elevator.setManualSpeed(speed); // Set the manual speed of the elevator
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setManualSpeed(0); // Stop elevator movement
    elevator.setManualControl(false); // Disable manual control
    // elevator.hold(); // Hold the elevator's current position
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}