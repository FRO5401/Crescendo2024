// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class ArmForward extends Command {
  Arm armForward;
  boolean endCommand;

  /** Creates a new ArmForward. */
  public ArmForward(Arm m_armForward) {
    // Use addRequirements() here to declare subsystem dependencies.
  armForward = m_armForward;
  addRequirements(armForward);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armForward.armForward();
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
