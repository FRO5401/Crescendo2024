// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//WPI Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.BackShooter;
import frc.robot.Commands.ClimberMove;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.ShiftGear;
import frc.robot.Commands.ShooterBack;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
//Command Imports
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.Auto.AutoAmpShot;
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.Auto.DefensiveAuto;
import frc.robot.Commands.Auto.FourPieceAuto;
import frc.robot.Commands.Auto.JustShoot;
import frc.robot.Commands.Auto.LimitSwitchCommand;
import frc.robot.Commands.Auto.OnePieceAuto;
import frc.robot.Commands.Auto.SideAuto;
import frc.robot.Commands.Auto.SideTwoPiece;
import frc.robot.Commands.Auto.ThreePieceAuto;
import frc.robot.Commands.Auto.ThreePieceFlipped;
import frc.robot.Commands.Auto.TwoPieceAuto;
import frc.robot.Commands.Lights.BlueLED;
import frc.robot.Commands.Lights.GreenLED;
import frc.robot.Commands.Lights.PurpleLED;
import frc.robot.Commands.Lights.RainbowLED;
import frc.robot.Commands.Lights.YellowLED;
//Subsystem Imports
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.Photonvision;
import frc.robot.Subsystems.Shooter;

//Used Imports that might be used in future
//import frc.robot.Commands.StopPivot;

public class RobotContainer {
    private final CommandXboxController operator = Controls.operator;
    private final CommandXboxController driver = Controls.driver;

    /* Drivebase */
    private static final Drivebase drivebase = new Drivebase();


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
    /* Camera */
    
    private final Photonvision backCamera = new Photonvision("Back");
     private final Photonvision frontCamera = new Photonvision("Front");


    private final Trigger hasNote = new Trigger(infeed::getLimitSwitch);

    private final LEDSubsystem LED = new LEDSubsystem();

    private final XboxMove xboxMove = new XboxMove(drivebase, backCamera, frontCamera);


 


  public RobotContainer() {
    drivebase.setDefaultCommand(xboxMove);

    leftClimber.setDefaultCommand(new ClimberMove(leftClimber, "Left"));
    rightClimber.setDefaultCommand(new ClimberMove(rightClimber, "Right"));
    

    drivebase.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));

    configureBindings();
    chooseAuto();
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
    operator.rightTrigger().onTrue(new Expel(infeed));
    //If "Left Trigger" pressed/held on operator controller intake command used (sucks note into infeed)
    operator.leftTrigger().onTrue(new Intake(infeed));
    //If "Left Bumper" pressed/held on operator controller stop intake command used (stops infeed)
    operator.leftBumper().onTrue(new StopAll(infeed, shooter));

    /** Pivot Commands */
    //If "Y" pressed/held on operator controller rotatepivotground command used (moves intake to ground)
    operator.y().whileTrue(new SequentialCommandGroup(new RotatePivotGround(infeed), new Intake(infeed)));
    //If "A" pressed/held on operator controller rotatepivotshooter command used (moves intake to shooter)
    operator.a().whileTrue(new RotatePivotShooter(infeed));
    //If "B" pressed/held on operator controller rotatepivotAir command used (moves intake to air)
    operator.b().whileTrue(new RotatePivotAir(infeed));
    operator.x().whileTrue(new ParallelCommandGroup(new ShooterBack(shooter), new Intake(infeed)));

    //Auto Shooter Methods.
    operator.povUp().onTrue(new ParallelCommandGroup( new BlueLED(LED),new AutoShoot(infeed, shooter)));
    operator.povDown().onTrue(new ParallelCommandGroup(new AmpShot(shooter), new BlueLED(LED)));
    operator.povLeft().onTrue(new SpeakerShot(shooter));
    

    driver.start().onTrue(new ShiftGear(drivebase));

    //AUTO TARGETING

    //LED COMMANDS
  
    driver.povDown().onTrue(new RainbowLED(LED));
    driver.povLeft().onTrue(new BlueLED(LED));
    driver.povUp().onTrue(new YellowLED(LED));
    driver.povRight().onTrue(new PurpleLED(LED));


    //Limit Switch  stop infeed
    hasNote.onFalse(new SequentialCommandGroup(new ParallelCommandGroup(new RotatePivotShooter(infeed)), new LimitSwitchCommand(infeed, shooter), new GreenLED(LED)));
/* 
    driver.povDown().onTrue(drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driver.povLeft().onTrue(drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driver.povUp().onTrue(drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    driver.povRight().onTrue(drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
*/

  }

  public void chooseAuto(){
    

    chooser.addOption("One Piece", new OnePieceAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Two Piece", new TwoPieceAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Three Piece", new ThreePieceAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Four Piece", new FourPieceAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Flipped Three Piece", new ThreePieceFlipped(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("One Piece Side", new SideAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("TwoPieceSide", new SideTwoPiece(drivebase, infeed, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("Peddie Edition", new JustShoot(infeed, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    chooser.addOption("DefensiveAuto", new DefensiveAuto(infeed, shooter, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


    



    chooser.addOption("Do Nothing", Commands.print("Oops"));


    Shuffleboard.getTab("Autonomous").add(chooser);

  }

  public static void endgameRumble(){
    if (DriverStation.isEnabled()){
    if (DriverStationJNI.getMatchTime() <= 20 && DriverStationJNI.getMatchTime() >= 1 ){
      Controls.xbox_driver.setRumble(RumbleType.kBothRumble, 1);
      Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 1);
    }else {
      Controls.xbox_driver.setRumble(RumbleType.kBothRumble, 0);
      Controls.xbox_operator.setRumble(RumbleType.kBothRumble, 0);
    }
  }
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public static Drivebase getDrivebase() {
      return drivebase;
  }


}