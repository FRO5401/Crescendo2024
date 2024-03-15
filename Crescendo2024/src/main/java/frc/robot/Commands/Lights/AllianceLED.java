// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Lights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.LEDSubsystem;

public class AllianceLED extends Command {
  LEDSubsystem LED;
  private boolean endCommand = false;
  int r, g, b = 0;
  /** Creates a new AllianceLED. */
  public AllianceLED(LEDSubsystem m_LED) {
    
    LED = m_LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      b = 200;
    } else {
      r = 200;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LED.setLEDColor(r, g, b);
    LED.setData();
    Commands.waitSeconds(.1);
    endCommand = true;
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
