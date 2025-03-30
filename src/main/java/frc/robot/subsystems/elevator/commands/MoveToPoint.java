package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class MoveToPoint extends Command {
    private final Elevator elevator;
    private final double targetPosition;
    private final Command nextCommand;
    private final double proximityThreshold;
    private boolean nextCommandScheduled = false;

    // Original constructor for backward compatibility
    public MoveToPoint(Elevator elevator, double targetPosition) {
        this(elevator, targetPosition, null, 0.0);
    }
    
    // Constructor with next command
    public MoveToPoint(Elevator elevator, double targetPosition, Command nextCommand) {
        this(elevator, targetPosition, nextCommand, 3.0); // Default proximity threshold (3 rotations)
    }
    
    // Full constructor with configurable threshold
    public MoveToPoint(Elevator elevator, double targetPosition, Command nextCommand, double proximityThreshold) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        this.nextCommand = nextCommand != null ? nextCommand.withTimeout(0.5) : null; // Add 1-second timeout to next command
        this.proximityThreshold = proximityThreshold;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition);
        nextCommandScheduled = false;
    }

    @Override
    public void execute() {
        // Check if we're close enough to the target but haven't scheduled the next command yet
        if (!nextCommandScheduled && nextCommand != null) {
            double currentPosition = elevator.getPosition();
            double distanceToTarget = Math.abs(currentPosition - targetPosition);
            
            if (distanceToTarget <= proximityThreshold) {
                CommandScheduler.getInstance().schedule(nextCommand);
                nextCommandScheduled = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentState() == ElevatorState.AT_SETPOINT;
    }
}