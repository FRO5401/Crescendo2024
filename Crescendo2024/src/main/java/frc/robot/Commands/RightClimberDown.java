// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

// WPI imports
import edu.wpi.first.wpilibj2.command.Command;

// Subsystem imports
import frc.robot.Subsystems.Climber;

public class RightClimberDown extends Command {
  Climber rclimberDown;
  boolean endCommand = false;

  /** Creates a new Climber. */
  public RightClimberDown(Climber m_rclimberDown) {
    rclimberDown = m_rclimberDown;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rclimberDown);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rclimberDown.rightClimbDown();
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