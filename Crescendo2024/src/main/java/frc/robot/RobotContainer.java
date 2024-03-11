// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.JList.DropLocation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
//Subsystem Imports
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;

//Command Imports
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.BackShooter;
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
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.Auto.AutoTarget;
import frc.robot.Commands.Lights.AllianceLED;
import frc.robot.Commands.Lights.BlueLED;
import frc.robot.Commands.Lights.GreenLED;
import frc.robot.Commands.Lights.RainbowLED;
import frc.robot.Commands.Lights.YellowLED;
//Subsystem Imports
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.Photonvision;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

      private final CommandXboxController operator = Controls.operator;
    private final CommandXboxController driver = Controls.driver;

    /* Drivebase */
    private static final Drivebase drivebase = new Drivebase();
    private final XboxMove xboxMove = new XboxMove(drivebase);

    /* Auto chooser 
     * TODO: Change to choose your own auto
     */
    private final SendableChooser<Command> chooser = new SendableChooser<>();

    /*Intake */
    private final Infeed infeed = new Infeed();
    /*Shooter */
    private final Shooter shooter = new Shooter();
    /*Climbers */
    private final Climber leftClimber = new Climber(Constants.ClimberConstants.LEFTCLIMBER_ID, false, "Left", 8);
    private final Climber rightClimber = new Climber(Constants.ClimberConstants.RIGHTCLIMBER_ID, true, "Right", 7);
    /* Cameraa */
    private final static Photonvision backCamera = new Photonvision("Back");
    private final static Photonvision frontCamera = new Photonvision("Front");

    //trigger for the limit switch having the note
    private final Trigger hasNote = new Trigger(infeed::getLimitSwitch);
    //Leds, currently not working, hardware is unplugged
    private final LEDSubsystem LED = new LEDSubsystem();
    // new and improved XBOX move with cameras!!
    private final XboxMove xboxMove = new XboxMove(drivebase, backCamera, frontCamera);


 

  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);

    leftClimber.setDefaultCommand(new ClimberMove(leftClimber, "Left"));
    rightClimber.setDefaultCommand(new ClimberMove(rightClimber, "Right"));

    configureBindings();
  }

  private void configureBindings() {

    /** Operator Controls */
    /*
     * Low Power Shot - Bottom DPAD 
     * Speaker Shot - Up DPAD
     * Left climber - left joysticks
     * Right climber - right joystick
     * Expel - Right Trigger
     * Intake - Left Trigger
     * Rotate Pivot Air - B
     * Rotate Pivot Grounds - Y
     * Rotate Pivot Shooter - A
     * Rotate Pivot Safe - X
     * Stop All - Left Bumber
    */

    /** Driver Controls */
    /*
      * Rainbow LEDs - Back Button
      * Blue LEDs - X
      * Alliance LEDs - Y
      * Shift Gear - Start Button
      * Auto Target - A
    */

    /** Intake Commands */
    //If "Right Trigger" pressed/held on operator controller expel command used (removes note from infeed)
    operator.rightTrigger().whileTrue(new ParallelCommandGroup(new Expel(infeed),new BlueLED(LED) ));
    //If "Left Trigger" pressed/held on operator controller intake++++ command used (sucks note into infeed)
    operator.leftTrigger().whileTrue(new Intake(infeed));
    //If "Left Bumper" pressed/held on operator controller stop intake command used (stops infeed)
    operator.leftBumper().whileTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Y" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.y().whileTrue(new SequentialCommandGroup(new RotatePivotGround(infeed), new Intake(infeed)));
    //If "A" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.a().whileTrue(new RotatePivotShooter(infeed));
    //If "B" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.b().whileTrue(new RotatePivotAir(infeed));
    operator.x().whileTrue(new ParallelCommandGroup(new Intake(infeed), new BackShooter(shooter)));

    operator.povUp().onTrue(new ParallelCommandGroup(new AutoShoot(infeed, shooter), new BlueLED(LED)));
    operator.povDown().onTrue(new ParallelCommandGroup(new AmpShot(shooter), new BlueLED(LED)));
    operator.povLeft().onTrue(new SpeakerShot(shooter));
 
    driver.start().onTrue(new ShiftGear(drivebase));

    /*UNCOMMENT WHEN TUNING USING SYSID
     * driver.povDown().onTrue(new SequentialCommandGroup(new ShiftGear(drivebase),drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward)));
     * driver.povRight().onTrue(new SequentialCommandGroup(new ShiftGear(drivebase),drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse)));
     * driver.povLeft().onTrue(new SequentialCommandGroup(new ShiftGear(drivebase),drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward)));
     * driver.povUp().onTrue(new SequentialCommandGroup(new ShiftGear(drivebase),drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))); 
     */

    driver.povDown().onTrue(new RainbowLED(LED));
    driver.povLeft().onTrue(new BlueLED(LED));
    driver.povUp().onTrue(new YellowLED(LED));
    driver.povDown().onTrue(Commands.runOnce(()-> LED.setLEDColor(0, 0, 0), LED));


   hasNote.onTrue(new SequentialCommandGroup(new RotatePivotShooter(infeed), new StopAll(infeed, shooter), new GreenLED(LED)));
   
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}