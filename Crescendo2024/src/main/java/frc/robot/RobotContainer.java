// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Subsystem Imports
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;

//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.*;


public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;
    //Drivebase
    private final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /*Climber */
    private final Climber climber = new Climber();
 

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    configureBindings();
  }

  private void configureBindings() {
    //We'll change all the buttons later!
    operator.leftBumper().whileTrue(new LeftClimberUp(climber));
    operator.rightBumper().whileTrue(new RightClimberUp(climber));
    operator.leftTrigger().whileTrue(new LeftClimberDown(climber));
    operator.rightTrigger().whileTrue(new RightClimberDown(climber));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
