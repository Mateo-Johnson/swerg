package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.coral.Coral;

/**
 * A collection of commands related to the Coral subsystem, which controls the intake and eject actions.
 */
public class CoralCommands {

    /**
     * Creates an InstantCommand that runs the intake action on the Coral subsystem.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The InstantCommand that will run the intake method.
     */
    public static Command intake(Coral coral) {
        return new InstantCommand(
            () -> coral.intake(),
            coral
        );
    }

    /**
     * Creates an InstantCommand that runs the fast eject action on the Coral subsystem.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The InstantCommand that will run the fastEject method.
     */
    public static Command fastEject(Coral coral) {
        return new InstantCommand(
            () -> coral.fastEject(),
            coral
        );
    }

    /**
     * Creates an InstantCommand that runs the slow eject action on the Coral subsystem.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The InstantCommand that will run the slowEject method.
     */
    public static Command slowEject(Coral coral) {
        return new InstantCommand(
            () -> coral.slowEject(),
            coral
        );
    }

    /**
     * Creates an InstantCommand that runs the hold action on the Coral subsystem.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The InstantCommand that will run the hold method.
     */
    public static Command hold(Coral coral) {
        return new InstantCommand(
            () -> coral.hold(),
            coral
        );
    }

    /**
     * Creates an InstantCommand that stops the Coral subsystem.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The InstantCommand that will stop the Coral subsystem.
     */
    public static Command stop(Coral coral) {
        return new InstantCommand(
            () -> coral.stop(),
            coral
        );
    }

    /**
     * Creates a command that either intakes or fast ejects based on whether a game piece is present.
     * This command checks if the Coral subsystem has a game piece. If it does, it performs a fast eject; 
     * otherwise, it runs the intake action.
     * 
     * @param coral The Coral subsystem to be used.
     * @return The Command that runs the appropriate action based on the game piece status.
     */
    public static Command intakeEject(Coral coral) {
        return new Command() {
            @Override
            public void initialize() {
                if (coral.hasGamePiece()) {
                    coral.fastEject();
                } else {
                    coral.intake();
                }
            }
            
            @Override
            public void execute() {
                // Command continues running the selected mode
            }
            
            @Override
            public boolean isFinished() {
                // Check if the command is finished based on the current mode
                if (coral.getCurrentMode() == Coral.IntakeMode.INTAKING) {
                    return coral.hasGamePiece();
                } else if (coral.getCurrentMode() == Coral.IntakeMode.FAST_EJECTING) {
                    return coral.isAtTargetVelocity();
                }
                return false;
            }
            
            @Override
            public void end(boolean interrupted) {
                // End the command by holding or stopping based on game piece status
                if (coral.hasGamePiece()) {
                    coral.hold();
                } else {
                    coral.stop();
                }
            }
        }.withTimeout(2.0) // Optional timeout as a safety measure
         .withName("IntakeEject");
    }
}

//DEFAULT COMMAND
//coral.setDefaultCommand(CoralCommands.hold(coral));

//operatorController.a().whileTrue(CoralCommands.intake(coral));
//operatorController.b().whileTrue(CoralCommands.fastEject(coral));
//operatorController.x().whileTrue(CoralCommands.slowEject(coral));