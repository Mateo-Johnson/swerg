package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * A command that homes the elevator, moving it to the lower limit.
 */
public class Home extends Command {
  private final Elevator elevator;
  private static final double TIMEOUT_SECONDS = 5.0;

  /**
   * Creates a new HomeCommand.
   * 
   * @param elevator The Elevator subsystem to be homed.
   */
  public Home(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    setName("Home");
    // Add timeout for safety
    withTimeout(TIMEOUT_SECONDS);
  }

  @Override
  public void initialize() {
    elevator.startHoming(); // Start homing procedure
  }

  @Override
  public void execute() {
    elevator.completeHoming(); // Complete homing procedure
  }

  @Override
  public boolean isFinished() {
    // Command finishes when elevator reaches the lower limit, is stalled, or encounters an error
    return elevator.isAtLowerLimit() ||
      elevator.getCurrentState() == ElevatorState.STALLED ||
      elevator.getCurrentState() == ElevatorState.ERROR;
  }

  @Override
  public void end(boolean interrupted) {
    // Clear any errors or handle interruption
    if (interrupted ||
      elevator.getCurrentState() == ElevatorState.STALLED ||
      elevator.getCurrentState() == ElevatorState.ERROR) {
      elevator.clearError();
    }
  }
}