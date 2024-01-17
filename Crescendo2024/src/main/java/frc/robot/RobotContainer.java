// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.XboxMove;
import frc.robot.Subsystems.Drivebase;

public class RobotContainer {

  private final Drivebase driveBase = new Drivebase();
  private final XboxMove xboxMove = new XboxMove(driveBase);

  public RobotContainer() {

    driveBase.setDefaultCommand(xboxMove);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
