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

    /*Climbers */
    private final Climber leftClimber = new Climber(Constants.ClimberConstants.LEFTCLIMBER_ID, true, "Left");
    private final Climber rightClimber = new Climber(Constants.ClimberConstants.RIGHTCLIMBER_ID, false, "Right");
 

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    configureBindings();
  }

  private void configureBindings() {
    // Left climber controlls
    operator.leftBumper().whileTrue(new ClimberUp(leftClimber));
    operator.leftTrigger().whileTrue(new ClimberDown(leftClimber));

    // Right climber controlls
    operator.rightBumper().whileTrue(new ClimberUp(rightClimber));
    operator.rightTrigger().whileTrue(new ClimberDown(rightClimber));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
