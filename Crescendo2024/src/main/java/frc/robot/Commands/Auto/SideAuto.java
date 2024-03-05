// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.RotatePivotAir;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SideAuto extends SequentialCommandGroup {
  Infeed infeed;
  Shooter shooter;
  Drivebase drivebase;


  Trajectory leaveStart;


  int flipped;
  /** Creates a new ThreePieceAuto. */
  public SideAuto(  Infeed m_infeed, Shooter m_shooter, Drivebase m_drivebase) {
    infeed = m_infeed;
    shooter = m_shooter;
    drivebase = m_drivebase;

    if(DriverStation.getAlliance().get() == Alliance.Blue){
      flipped = -1;
    } else {
      flipped = 1;
    }

        leaveStart = TrajectoryGenerator.generateTrajectory(

        // Start at the origin facing the +X direction

        drivebase.getPose(),

        // Pass through these two interior waypoints, making an 's' curve path

        List.of(new Translation2d(.5, flipped*.25)),

        // End 3 meters straight ahead of where we started, facing forward

        new Pose2d(1.3, flipped*.30, new Rotation2d(0)),

            // Pass config

        Constants.AutoConstants.config);
    addCommands(
      new AutoShoot(infeed, shooter),

      new RotatePivotAir(infeed),

      new RamseteCommand(

            leaveStart,

            drivebase::getPose,

            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),

            new SimpleMotorFeedforward(

                Constants.AutoConstants.ksVolts,

                Constants.AutoConstants.kvVoltSecondsPerMeter,

                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),

            Constants.AutoConstants.kDriveKinematics,

            drivebase::getWheelSpeeds,

            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),

            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),

            // RamseteCommand passes volts to the callback

            drivebase::tankDriveVolts,

            drivebase)

);

  }
}
