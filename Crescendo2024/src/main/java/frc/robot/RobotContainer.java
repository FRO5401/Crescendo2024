// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//Subsystem Imports
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
import frc.robot.Commands.TrapShot;

public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;

    /* Drivebase */
    private final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /*Intake */
    private final Infeed infeed = new Infeed();

    /*Shooter */
    private final Shooter shooter = new Shooter();
 

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
    //If "Right Bumper" pressed/held on operator controller stop all command used (stops infeed and shooter)
    operator.rightBumper().whileTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Right Trigger" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.rightTrigger().whileTrue(new RotatePivotGround(infeed));
    //If "Left Trigger" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.leftTrigger().whileTrue(new RotatePivotShooter(infeed));
    //If "Left Bumper" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.leftBumper().whileTrue(new RotatePivotAir(infeed));

    /** Shooter Commands */
    //If "D-pad Up" pressed/held on operator controller SpeakerShot command used (Shoots note into speaker)
    operator.povUp().onTrue(new SpeakerShot(shooter));
    //If "D-pad Right" pressed/held on operator controller AmpShot command used (Shoots note into Amp)
    operator.povRight().onTrue(new AmpShot(shooter));
    //If "D-pad Down" pressed/held on operator controller TrapShot command used (Shoots note into trap)
    operator.povDown().onTrue(new TrapShot(shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}