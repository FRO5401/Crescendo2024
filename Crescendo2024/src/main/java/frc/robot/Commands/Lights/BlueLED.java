// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Lights;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;

//File Imports
import frc.robot.Subsystems.LEDSubsystem;

public class BlueLED extends Command {
  //Declaring Variables
  private LEDSubsystem LED;

  /** Creates a new BlueLED. */
  public BlueLED(LEDSubsystem m_LED) {

    LED=m_LED;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LED.setLEDColor(0, 0, 225);
    LED.setData();
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
