// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LearnAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// SUbsystem imports
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
  Infeed infeed;
  Shooter shooter;
  /** Creates a new LearnAuto. */
  public PickupShoot(Infeed m_infeed, Shooter m_shooter) {
    infeed = m_infeed;
    shooter = m_shooter;

    addRequirements(infeed, shooter);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new RotatePivotGround(infeed),

      new Intake(infeed),

      new StopAll(infeed, shooter),

      new RotatePivotShooter(infeed),

      new WaitCommand(1),

      new AutoShoot(infeed, shooter),

      new WaitCommand(2),

      new StopAll(infeed, shooter)
    );
  }
}
