// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LearnAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//    WPI Imports
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
//    Subsystems Imports
import frc.robot.Subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class S_Curve extends SequentialCommandGroup {
  Drivebase drivebase;

  Trajectory Curve_S;
  Trajectory leaveStart;

  /** Creates a new S_Curve. */
  public S_Curve(Drivebase m_drivebase) {

    drivebase = m_drivebase;

    //Generating trajectitory
    Curve_S = TrajectoryGenerator.generateTrajectory(
    
    drivebase.getPose(),

    List.of(new Translation2d(2, 1)),

    new Pose2d(1.1, 0, new Rotation2d(0)),
    
    AutoConstants.config);



    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
