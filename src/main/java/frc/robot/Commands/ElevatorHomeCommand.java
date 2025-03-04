// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ElevatorHomeCommand extends Command {
  private final PivotSubsystem pivot;
 private final ElevatorSubsystem elevator; 
  /** Creates a new AmpShot. */
  public ElevatorHomeCommand(PivotSubsystem pivot,ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.elevator = elevator;
    addRequirements(pivot,elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // pivot.ampPositionCommand();
   // elevator.ampPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


}
