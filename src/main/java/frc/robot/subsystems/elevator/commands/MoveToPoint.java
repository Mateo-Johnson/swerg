package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/**
 * Command to move the elevator to a specified position.
 */
public class MoveToPoint extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    /**
     * Creates a new MoveToPoint command.
     *
     * @param elevator The elevator subsystem used by this command.
     * @param targetPosition The target position in rotations.
     */
    public MoveToPoint(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        // The elevator subsystem handles the movement in its periodic method
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        // Check if the elevator has reached the target position
        return Math.abs(elevator.getPosition() - targetPosition) < 0.1; // Adjust tolerance as needed
    }
}
