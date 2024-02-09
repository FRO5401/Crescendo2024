// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Commands;

// WPI imports
import edu.wpi.first.wpilibj2.command.Command;

// Subsystem imports
import frc.robot.Subsystems.Climber;

public class LeftClimberDown extends Command {
  Climber lclimberDown;
  boolean endCommand = false;

  /** Creates a new Climber. */
  public LeftClimberDown(Climber m_lclimberDown) {
    lclimberDown = m_lclimberDown;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lclimberDown);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lclimberDown.leftClimbDown();
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