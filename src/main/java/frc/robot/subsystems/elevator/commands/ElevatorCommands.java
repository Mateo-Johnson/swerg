package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ElevatorCommands {
    // Manual control command that runs while held
    public static Command elevatorManualMove(Elevator elevator, double speed) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.setManualControl(true);
            }

            @Override
            public void execute() {
                elevator.setManualSpeed(speed);
            }

            @Override
            public void end(boolean interrupted) {
                elevator.setManualControl(false);
                elevator.setManualSpeed(0);
                elevator.hold(); 
            }

            @Override
            public boolean isFinished() {
                return false; // Run until interrupted
            }
        }.withName("ManualMove");
    }

    // Homing command that runs until the elevator reaches the lower limit
    public static Command elevatorHome(Elevator elevator) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.startHoming();
            }

            @Override
            public void execute() {
                elevator.completeHoming();
            }

            @Override
            public boolean isFinished() {
                return elevator.isAtLowerLimit() || 
                       elevator.getCurrentState() == ElevatorState.STALLED ||
                       elevator.getCurrentState() == ElevatorState.ERROR;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted || 
                    elevator.getCurrentState() == ElevatorState.STALLED || 
                    elevator.getCurrentState() == ElevatorState.ERROR) {
                    elevator.clearError();
                }
            }
        }.withTimeout(5.0) // Timeout after 5 seconds for safety
         .withName("Home");
    }

    // Command to move to a specific setpoint
    public static Command elevatorMoveToSetpoint(Elevator elevator, int setpointIndex) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.moveToSetpoint(setpointIndex);
            }

            @Override
            public boolean isFinished() {
                return elevator.atSetpoint() || 
                       elevator.getCurrentState() == ElevatorState.STALLED ||
                       elevator.getCurrentState() == ElevatorState.ERROR;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted || 
                    elevator.getCurrentState() == ElevatorState.STALLED || 
                    elevator.getCurrentState() == ElevatorState.ERROR) {
                    elevator.clearError();
                }
            }
        }.withTimeout(5.0) // Timeout after 5 seconds for safety
         .withName("MoveToSetpoint_" + setpointIndex);
    }

    // Convenience methods for specific setpoints
    public static Command moveToL1(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 0);
    }

    public static Command moveToL2(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 1);
    }

    public static Command moveToL3(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 2);
    }

    public static Command moveToL4(Elevator elevator) {
        return elevatorMoveToSetpoint(elevator, 3);
    }

        /**
     * Creates a default command that holds the elevator's position.
     * This command will run whenever no other command is using the elevator.
     */
    public static Command elevatorHold(Elevator elevator) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.hold();
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

//ROBOTCONTAINER
//elevator.setDefaultCommand(ElevatorCommands.elevatorHold(elevator));

//operatorController.a().whileTrue(ElevatorCommands.moveToL1(elevator));
//operatorController.b().whileTrue(ElevatorCommands.elevatorHomeCommand(elevator));
//operatorController.x().whileTrue(ElevatorCommands.elevatorManualMove(elevator, -1));