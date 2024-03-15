// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.AmpShot;
import frc.robot.Commands.Expel;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAmpShot extends SequentialCommandGroup {
  private Shooter shooter;
  private Infeed infeed;
  /** Creates a new AutoAmpShot. */
  public AutoAmpShot(Shooter m_shooter, Infeed m_infeed) {
    infeed = m_infeed;
    shooter = m_shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AmpShot(shooter),

      new WaitCommand(0.3),
      
      new Expel(infeed)
    );
  }
}
