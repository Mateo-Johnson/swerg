package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

/**
 * A collection of commands related to the Elevator subsystem that allows manual control, homing, and setpoint movement.
 */
public class ElevatorCommands {

    /**
     * Creates a command that allows manual control of the elevator. This command runs as long as it is held.
     * 
     * @param elevator The Elevator subsystem to be controlled.
     * @param speed The speed at which to move the elevator (positive for upward, negative for downward).
     * @return The Command that controls the elevator manually.
     */
    public static Command elevatorManualMove(Elevator elevator, double speed) {
        return new Command() {
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
                elevator.setManualControl(false); // Disable manual control
                elevator.setManualSpeed(0); // Stop elevator movement
                elevator.hold(); // Hold the elevator's current position
            }

            @Override
            public boolean isFinished() {
                return false; // Runs until interrupted
            }
        }.withName("ManualMove");
    }

    /**
     * Creates a command that homes the elevator, moving it to the lower limit.
     * 
     * @param elevator The Elevator subsystem to be homed.
     * @return The Command that runs the homing procedure.
     */
    public static Command elevatorHome(Elevator elevator) {
        return new Command() {
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
        }.withTimeout(5.0) // Timeout after 5 seconds for safety
         .withName("Home");
    }

    /**
     * Creates a command that moves the elevator to a specific setpoint.
     * 
     * @param elevator The Elevator subsystem to be moved.
     * @param setpointIndex The index of the setpoint to move the elevator to.
     * @return The Command that moves the elevator to the specified setpoint.
     */
    public static Command elevatorMoveToSetpoint(Elevator elevator, int setpointIndex) {
        return new Command() {
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
        }.withTimeout(5.0) // Timeout after 5 seconds for safety
         .withName("MoveToSetpoint_" + setpointIndex);
    }

    /**
     * Convenience method for moving the elevator to setpoint 1 (L1).
     * 
     * @param elevator The Elevator subsystem to be moved.
     * @return The Command that moves the elevator to setpoint L1.
     */
    public static Command moveToL1(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 0);
    }

    /**
     * Convenience method for moving the elevator to setpoint 2 (L2).
     * 
     * @param elevator The Elevator subsystem to be moved.
     * @return The Command that moves the elevator to setpoint L2.
     */
    public static Command moveToL2(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 1);
    }

    /**
     * Convenience method for moving the elevator to setpoint 3 (L3).
     * 
     * @param elevator The Elevator subsystem to be moved.
     * @return The Command that moves the elevator to setpoint L3.
     */
    public static Command moveToL3(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 2);
    }

    /**
     * Convenience method for moving the elevator to setpoint 4 (L4).
     * 
     * @param elevator The Elevator subsystem to be moved.
     * @return The Command that moves the elevator to setpoint L4.
     */
    public static Command moveToL4(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 3);
    }

    /**
     * Creates a default command that holds the elevator's position.
     * This command will run whenever no other command is using the elevator.
     * 
     * @param elevator The Elevator subsystem to be held.
     * @return The Command that holds the elevator in its current position.
     */
    public static Command elevatorHold(Elevator elevator) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.hold(); // Hold the elevator in its current position
            }

            @Override
            public void execute() {
                // Continuously update hold position if we're in manual control
                if (elevator.getCurrentState() == Elevator.ElevatorState.MANUAL_CONTROL) {
                    elevator.hold();
                }
            }

            @Override
            public boolean isFinished() {
                return false; // Never finish - this is a default command
            }
        }.withName("DefaultHold");
    }
}


// DEFAULT COMMAND
//elevator.setDefaultCommand(ElevatorCommands.elevatorHold(elevator));

//operatorController.a().whileTrue(ElevatorCommands.moveToL1(elevator));
//operatorController.b().whileTrue(ElevatorCommands.elevatorHomeCommand(elevator));
//operatorController.x().whileTrue(ElevatorCommands.elevatorManualMove(elevator, -1));