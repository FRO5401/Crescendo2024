// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//File Imports
import frc.robot.Subsystems.Infeed;

public class StopPivot extends Command {
  Infeed stoppivot;
  boolean endCommand = false;

  /** Creates a new Pivot. */
  public StopPivot(Infeed m_stoppivot) {
    stoppivot = m_stoppivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stoppivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    stoppivot.stopPivot();
    endCommand = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
