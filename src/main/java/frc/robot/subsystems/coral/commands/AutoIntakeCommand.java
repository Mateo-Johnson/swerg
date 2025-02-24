package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class AutoIntakeCommand extends Command {
  private final Coral coral;

  public AutoIntakeCommand(Coral coral) {
    this.coral = coral;
    addRequirements(coral);
  }

  @Override
  public void execute() {
    if (coral.hasGamePiece()) {
      coral.fastEject();
    } else {
      coral.intake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    coral.stop();
  }
}