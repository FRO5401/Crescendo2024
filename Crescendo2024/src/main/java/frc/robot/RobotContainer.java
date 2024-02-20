// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//WPI Imports
// WPI imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.ClimberMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotSafe;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.ShiftGear;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.Auto.AutoTarget;
//Subsystem Imports
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Photonvision;
import frc.robot.Subsystems.Shooter;

//Used Imports that might be used in future
//import frc.robot.Commands.StopPivot;

public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;
    private final CommandXboxController driver = Controls.driver;

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
    /* Camera */
    
    private final Photonvision camera = new Photonvision("Test");
 

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);
    leftClimber.setDefaultCommand(new ClimberMove(leftClimber, "Left"));
    rightClimber.setDefaultCommand(new ClimberMove(rightClimber, "Right"));
    configureBindings();
  }

  private void configureBindings() {

    /*
     * Low Power Shot - Bottom DPAD 
     * Speaker Shot - Up DPAD
     * Left climber - left joysticks
     * right climber - right joystick
     * Expel - Right Trigger
     * Intake - Left Trigger
     * Rotate Pivot Air - B
     * Rotate Pivot Grouns - Y
     * Rotate Pivot Shooter - A
     * Stop All - Left Bumber
     */

    /** Intake Commands */
    //If "Right Trigger" pressed/held on operator controller expel command used (removes note from infeed)
    operator.rightTrigger().whileTrue(new Expel(infeed));
    //If "Left Trigger" pressed/held on operator controller intake command used (sucks note into infeed)
    operator.leftTrigger().whileTrue(new Intake(infeed));
    //If "Left Bumper" pressed/held on operator controller stop intake command used (stops infeed)
    operator.leftBumper().whileTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Y" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.y().whileTrue(new RotatePivotGround(infeed));
    //If "A" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.a().whileTrue(new RotatePivotShooter(infeed));
    //If "B" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.b().whileTrue(new RotatePivotAir(infeed));
    operator.x().whileTrue(new RotatePivotSafe(infeed));


    operator.povUp().onTrue(new AutoShoot(infeed, shooter));
    operator.povDown().onTrue(new AmpShot(shooter));

    driver.start().onTrue(new ShiftGear(drivebase));
    driver.a().whileTrue(new AutoTarget(camera, drivebase, 1, 0));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}