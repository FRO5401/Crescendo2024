// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//FIle Imports
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class TrapShot extends Command {
  //Declaring Variables
  private Shooter shooter;

  /** Creates a new SpeakerShot. */
  public TrapShot(Shooter m_shooter) {

    shooter = m_shooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(Constants.ShooterConstants.SpeedConstants.SHOOTER_RPM);
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
