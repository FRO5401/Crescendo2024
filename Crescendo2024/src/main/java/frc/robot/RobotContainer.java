// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.XboxMove;
import frc.robot.Subsystems.Drivebase;

public class RobotContainer {
  // Defines the drivebase
  private final Drivebase drivebase = new Drivebase();
  // Tells xboxMove which drivebase it can control
  private final XboxMove xboxMove = new XboxMove(drivebase);

  public RobotContainer() {

drivebase.setDefaultCommand(xboxMove);



    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
