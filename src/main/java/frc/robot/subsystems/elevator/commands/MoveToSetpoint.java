package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/**
 * A command that moves the elevator to a specified height (in rotations).
 */
public class MoveToSetpoint extends Command {
  private final Elevator elevator;
  private final double targetHeight;
  private boolean isFinished = false;
  
  /**
   * Creates a new MoveElevatorToHeight command.
   *
   * @param elevator The elevator subsystem.
   * @param targetHeight The target height in rotations.
   */
  public MoveToSetpoint(Elevator elevator, double targetHeight) {
    this.elevator = elevator;
    this.targetHeight = targetHeight;
    
    // This command requires the elevator subsystem
    addRequirements(elevator);
  }
  
  @Override
  public void initialize() {
    // Ensure we're not in manual control mode
    elevator.setManualControl(false);
    
    // Set the target position
    elevator.setTargetPosition(targetHeight);
    
    isFinished = false;
  }
  
  @Override
  public void execute() {
    // The elevator's periodic method handles the movement
    // Just check if we've reached the target
    if (elevator.atSetpoint()) {
      isFinished = true;
    }
    
    // Also check for error states
    if (elevator.getCurrentState() == Elevator.ElevatorState.ERROR || 
        elevator.getCurrentState() == Elevator.ElevatorState.STALLED ||
        elevator.getCurrentState() == Elevator.ElevatorState.ENCODER_ERROR) {
      isFinished = true;
    }
  }
  
  @Override
  public boolean isFinished() {
    return isFinished;
  }
  
  @Override
  public void end(boolean interrupted) {
    // If the command was interrupted, the elevator will continue to the position
    // No need to stop it or change its state
    if (interrupted) {
      return;
    }
    
    // If we reached the setpoint naturally, make sure we're in IDLE state
    if (elevator.atSetpoint()) {
      elevator.setState(Elevator.ElevatorState.IDLE);
    }
  }
}