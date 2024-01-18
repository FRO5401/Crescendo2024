// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ArmBackward;
import frc.robot.Commands.ArmForward;
import frc.robot.Commands.Pickup;
import frc.robot.Commands.Putdown;
import frc.robot.Commands.XboxMove;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Intake;


public class RobotContainer {
  private final CommandXboxController operator = Controls.operator;
  // Defines the drivebase
  private final Drivebase drivebase = new Drivebase();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  // Tells xboxMove which drivebase it can control
  private final XboxMove xboxMove = new XboxMove(drivebase);

  //For pickup putdown
  private final Pickup pickup = new Pickup(intake);
  private final Putdown putdown = new Putdown(intake);
  //for Armforward
  private final ArmForward armforward = new ArmForward(arm);
  private final ArmBackward armbackward = new ArmBackward(arm);


  public RobotContainer() {

drivebase.setDefaultCommand(xboxMove);



    configureBindings();
  }

  private void configureBindings() {
    
    operator.a().whileTrue(pickup);
    operator.b().whileTrue(putdown);

    operator.x().whileTrue(armforward);
    operator.y().whileTrue(armbackward);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
