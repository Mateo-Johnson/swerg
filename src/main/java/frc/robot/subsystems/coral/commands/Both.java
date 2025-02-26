package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class Both extends Command {
  private final Coral coral;
  private final double intakeSpeed;
  private final double triggerDistance;
  
  // Static flag to track if this is the first execution or a subsequent one
  private static boolean hasTriggeredOnce = false;
  
  // Flag to indicate if this execution should check sonar or just run until canceled
  private boolean checkSonar;

  /**
   * Creates a command that runs the coral intake until the sonar detects an object.
   * On first execution, it stops when sonar is triggered.
   * On subsequent executions, it runs until the command is canceled.
   * 
   * @param coral The coral subsystem
   * @param intakeSpeed The speed to run the intake (positive value)
   * @param triggerDistance The distance in inches at which to stop the intake
   */
  public Both(Coral coral, double intakeSpeed, double triggerDistance) {
    this.coral = coral;
    this.intakeSpeed = Math.abs(intakeSpeed); // Ensure positive value
    this.triggerDistance = triggerDistance;
    
    // Register requirements
    addRequirements(coral);
  }

  /**
   * Creates a command using a default trigger distance of 2.0 inches.
   * 
   * @param coral The coral subsystem
   * @param intakeSpeed The speed to run the intake (positive value)
   */
  public Both(Coral coral, double intakeSpeed) {
    this(coral, intakeSpeed, 2.0);
  }

  @Override
  public void initialize() {
    // Determine if this execution should check sonar or just run
    checkSonar = !hasTriggeredOnce;
    
    // Start the intake when the command begins
    coral.forward(intakeSpeed);
  }

  @Override
  public void execute() {
    // Nothing needed here as the subsystem's periodic method updates the motors
  }

  @Override
  public boolean isFinished() {
    if (checkSonar) {
      // If we're checking the sonar and it's triggered, mark that we've triggered once
      if (coral.getSonarDistance() <= triggerDistance) {
        hasTriggeredOnce = true;
        return true;
      }
      return false;
    } else {
      // If not checking sonar, don't finish automatically (will be canceled when button is released)
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Only stop the motors if we're ending because of sonar trigger
    // or if explicitly interrupted (button released)
    coral.stop();
    
    // If the command was interrupted (button released), reset the trigger flag
    if (interrupted) {
      hasTriggeredOnce = false;
    }
  }
}