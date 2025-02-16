package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.coral.Coral;

public class CoralCommands {
    // Instant commands for different coral states
    public static Command intakeCommand(Coral coral) {
        return new InstantCommand(
            () -> coral.intake(),
            coral
        );
    }

    public static Command fastEjectCommand(Coral coral) {
        return new InstantCommand(
            () -> coral.fastEject(),
            coral
        );
    }

    public static Command slowEjectCommand(Coral coral) {
        return new InstantCommand(
            () -> coral.slowEject(),
            coral
        );
    }

    public static Command holdCommand(Coral coral) {
        return new InstantCommand(
            () -> coral.hold(),
            coral
        );
    }

    public static Command stopCommand(Coral coral) {
        return new InstantCommand(
            () -> coral.stop(),
            coral
        );
    }
}

//DEFAULT COMMAND
//coral.setDefaultCommand(CoralCommands.holdCommand(coral));

//operatorController.a().whileTrue(CoralCommands.intakeCommand(coral));
//operatorController.b().whileTrue(CoralCommands.fastEjectCommand(coral));
//operatorController.x().whileTrue(CoralCommands.slowEjectCommand(coral));