package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class MoveToPoint extends Command {
    private final Elevator elevator;
    private final double targetPosition;
    private final Command nextCommand;
    private final boolean returnToZero;
    private final double returnDelay; // Delay in seconds before returning to zero
    private static final double proxThreshold = 3.0; // Fixed proximity threshold
    private boolean nextCommandScheduled = false;
    private boolean returnCommandScheduled = false;

    // Original constructor for backward compatibility
    public MoveToPoint(Elevator elevator, double targetPosition) {
        this(elevator, targetPosition, null, false, 0.0);
    }

    // Constructor with next command
    public MoveToPoint(Elevator elevator, double targetPosition, Command nextCommand) {
        this(elevator, targetPosition, nextCommand, false, 0.0); // No return to zero by default
    }

    // Constructor with return to zero option
    public MoveToPoint(Elevator elevator, double targetPosition, Command nextCommand, boolean returnToZero) {
        this(elevator, targetPosition, nextCommand, returnToZero, 1.0); // Default 1 second delay
    }

    // Complete constructor with all options
    public MoveToPoint(Elevator elevator, double targetPosition, Command nextCommand, boolean returnToZero, double returnDelay) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        this.nextCommand = nextCommand != null ? nextCommand.withTimeout(0.5) : null; // Add 0.5-second timeout to next command
        this.returnToZero = returnToZero;
        this.returnDelay = returnDelay;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition);
        nextCommandScheduled = false;
        returnCommandScheduled = false;
    }

    @Override
    public void execute() {
        // Check if we're close enough to the target but haven't scheduled the next command yet
        if (!nextCommandScheduled && nextCommand != null) {
            double currentPosition = elevator.getPosition();
            double distanceToTarget = Math.abs(currentPosition - targetPosition);

            if (distanceToTarget <= proxThreshold) {
                Command commandToRun = nextCommand;

                // If returnToZero is true, set up the return command to run after the next command
                if (returnToZero && !returnCommandScheduled) {
                    final Command returnCommand = new MoveToPoint(elevator, 0.0);

                    // Add delay before returning to zero
                    if (returnDelay > 0) {
                        commandToRun = commandToRun.andThen(new WaitCommand(returnDelay)).andThen(returnCommand);
                    } else {
                        commandToRun = commandToRun.andThen(returnCommand);
                    }

                    returnCommandScheduled = true;
                }

                CommandScheduler.getInstance().schedule(commandToRun);
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