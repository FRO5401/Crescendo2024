// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPI Imports
// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Subsystem Imports
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;

import frc.robot.Subsystems.Shooter;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
import frc.robot.Commands.TrapShot;

//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.*;

//Used Imports that might be used in future
//import frc.robot.Commands.StopPivot;

public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;

    /* Drivebase */
    private final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /*Intake */
  private final Infeed infeed = new Infeed();
    /*Shooter */
    private final Shooter shooter = new Shooter();
    /*Climbers */
    private final Climber leftClimber = new Climber(Constants.ClimberConstants.LEFTCLIMBER_ID, true, "Left");
    private final Climber rightClimber = new Climber(Constants.ClimberConstants.RIGHTCLIMBER_ID, false, "Right");
 

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    configureBindings();
  }

  private void configureBindings() {

    /** Intake Commands */
    //If "A" pressed/held on operator controller expel command used (removes note from infeed)
    operator.a().whileTrue(new Expel(infeed));
    //If "B" pressed/held on operator controller intake command used (sucks note into infeed)
    operator.b().whileTrue(new Intake(infeed));
    //If "Right Bumper" pressed/held on operator controller stop intake command used (stops infeed)
    operator.rightBumper().whileTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Right Trigger" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.rightTrigger().whileTrue(new RotatePivotGround(infeed));
    //If "Left Trigger" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.leftTrigger().whileTrue(new RotatePivotShooter(infeed));
    //If "Left Bumper" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.leftBumper().whileTrue(new RotatePivotAir(infeed));


    operator.povUp().onTrue(new SpeakerShot(shooter));
    operator.povRight().onTrue(new AmpShot(shooter));
    operator.povDown().onTrue(new TrapShot(shooter));
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