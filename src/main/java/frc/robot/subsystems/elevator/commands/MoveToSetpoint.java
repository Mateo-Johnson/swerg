package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * A command that moves the elevator to a specific setpoint.
 */
public class MoveToSetpoint extends Command {
  private final Elevator elevator;
  private final int setpointIndex;
  private static final double TIMEOUT_SECONDS = 5.0;

  /**
   * Creates a new MoveToSetpointCommand.
   * 
   * @param elevator The Elevator subsystem to be moved.
   * @param setpointIndex The index of the setpoint to move the elevator to.
   */
  public MoveToSetpoint(Elevator elevator, int setpointIndex) {
    this.elevator = elevator;
    this.setpointIndex = setpointIndex;
    addRequirements(elevator);
    setName("MoveToSetpoint_" + setpointIndex);
    // Add timeout for safety
    withTimeout(TIMEOUT_SECONDS);
  }

  @Override
  public void initialize() {
    elevator.moveToSetpoint(setpointIndex); // Move to specified setpoint
  }

  @Override
  public boolean isFinished() {
    // Command finishes when elevator reaches setpoint or encounters an error or stall
    return elevator.atSetpoint() ||
      elevator.getCurrentState() == ElevatorState.STALLED ||
      elevator.getCurrentState() == ElevatorState.ERROR;
  }

  @Override
  public void end(boolean interrupted) {
    // Handle interruptions or errors
    if (interrupted ||
      elevator.getCurrentState() == ElevatorState.STALLED ||
      elevator.getCurrentState() == ElevatorState.ERROR) {
      elevator.clearError();
    }
  }
}