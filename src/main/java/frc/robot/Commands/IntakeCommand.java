package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class IntakeCommand extends Command{

    private final CoralIntakeSubsystem intake;
    
    private boolean firstcheck = true;

    public IntakeCommand(CoralIntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);

      }

      @Override
  public void initialize() {
    intake.intake();  
  }

  @Override
  public void execute()
  {
    intake.intake();  
  }

      @Override
      public boolean isFinished() {
          return intake.coralCheck();
      }

     @Override
     public void end(boolean interrupted) {
      intake.stop();
    }
}