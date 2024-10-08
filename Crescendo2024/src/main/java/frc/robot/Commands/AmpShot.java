// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class AmpShot extends Command {
  private Shooter shooter;
  private boolean endCommand = false;

  /** Creates a new SpeakerShot. */
  public AmpShot(Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(Constants.ShooterConstants.SpeedConstants.AMP_RPM);

    if(shooter.getVelocity() > Constants.ShooterConstants.SpeedConstants.AMP_RPM && shooter.getVelocity() < Constants.ShooterConstants.SpeedConstants.AMP_RPM + 50){
      endCommand = true;
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
