// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Subsystem Imports
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.StopIntake;
import frc.robot.Commands.StopPivot;
//Command Imports
import frc.robot.Commands.XboxMove;


public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;
    //Drivebase
    private final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /*Intake */
  private final Infeed infeed = new Infeed();

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    configureBindings();
  }

  private void configureBindings() {

    operator.a().whileTrue(new Expel(infeed));
    operator.b().whileTrue(new Intake(infeed));
    operator.rightBumper().whileTrue(new StopIntake(infeed));

    operator.rightTrigger().whileTrue(new RotatePivotGround(infeed));
    operator.leftTrigger().whileTrue(new RotatePivotShooter(infeed));
    operator.leftBumper().whileTrue(new RotatePivotAir(infeed));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}