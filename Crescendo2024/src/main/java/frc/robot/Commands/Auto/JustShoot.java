// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.StopAll;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustShoot extends SequentialCommandGroup {
  /** Creates a new JustShoot. */
  Infeed infeed;
  Shooter shooter;
  public JustShoot(Infeed m_infeed, Shooter m_shooter) {
    infeed = m_infeed;
    shooter = m_shooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_infeed, m_shooter),
      new WaitCommand(0.5),
      new StopAll(m_infeed, m_shooter)
    );
  }
}
