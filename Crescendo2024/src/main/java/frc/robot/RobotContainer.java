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

//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.StopIntake;
import frc.robot.Commands.StopPivot;
import frc.robot.Commands.RotatePivotShooter;

public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;
    //Drivebase
    private final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /*Intake */
    private final Infeed infeed = new Infeed();
    //Infeed Commands
    private final Expel expel = new Expel(infeed);
    private final Intake intake = new Intake(infeed);
    private final StopIntake stopintake = new StopIntake(infeed);
    //Pivot Commands
    private final RotatePivotGround rotatetoground = new RotatePivotGround(infeed);
    private final RotatePivotShooter rotatetoshooter = new RotatePivotShooter(infeed);
    private final StopPivot stoppivot = new StopPivot(infeed);

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    configureBindings();
  }

  private void configureBindings() {

    operator.a().whileTrue(expel);
    operator.b().whileTrue(intake);
    operator.rightBumper().whileTrue(stopintake);

    operator.rightTrigger().whileTrue(rotatetoground);
    operator.leftTrigger().whileTrue(rotatetoshooter);
    operator.leftBumper().whileTrue(stoppivot);

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
