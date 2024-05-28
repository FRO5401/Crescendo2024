// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LearnAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Subsystem imports
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// Command imports
import frc.robot.Commands.Auto.AutoShoot;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.Intake;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupShoot extends SequentialCommandGroup {
  //    Create Subsystems
  Infeed infeed;
  Shooter shooter;
  /** Creates a new LearnAuto. */
  public PickupShoot(Infeed m_infeed, Shooter m_shooter) {
    //    Makes Subsystems local variable
    infeed = m_infeed;
    shooter = m_shooter;

    addRequirements(infeed, shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      // Moves infeed to ground
      new RotatePivotGround(infeed),

      //Infeed intakes note
      new Intake(infeed),

      //All subsytems stop
      new StopAll(infeed, shooter),

      //Infeed rotates to shooter
      new RotatePivotShooter(infeed),

      //Waits before next command
      new WaitCommand(1),

      //Shoots note
      new AutoShoot(infeed, shooter),

      //Wait before stopping everything
      new WaitCommand(2),

      //Stops all subsystems
      new StopAll(infeed, shooter)
    );
  }
}
