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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.StopAll;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefensiveAuto extends SequentialCommandGroup {
  Infeed infeed;
  Shooter shooter;
  Drivebase drivebase;

  int flipped;

  /** Creates a new DefensiveAuto. */
  public DefensiveAuto(Infeed m_infeed, Shooter m_shooter, Drivebase m_drivebase) {
    infeed = m_infeed;
    shooter = m_shooter;
    drivebase = m_drivebase;

        if(DriverStation.getAlliance().get() == Alliance.Blue){
      flipped = -1;
    } else {
      flipped = 1;
    }

    Trajectory leaveStart = TrajectoryGenerator.generateTrajectory(

        // Start at the origin facing the +X direction

        new Pose2d(0, 0, new Rotation2d(0)),

        // Pass through these two interior waypoints, making an 's' curve path

        List.of(new Translation2d(2.5, flipped*-0.5)),

        // End 3 meters straight ahead of where we started, facing forward

        new Pose2d(8, flipped*-0.2, new Rotation2d(0)),

            // Pass config

        Constants.AutoConstants.config);

        Trajectory removeNote1 = TrajectoryGenerator.generateTrajectory(

        // Start at the origin facing the +X direction

        new Pose2d(8, flipped*-0.2, new Rotation2d(Units.degreesToRadians(90))),

        // Pass through these two interior waypoints, making an 's' curve path

        List.of(new Translation2d(8.25,  flipped*0)),

        // End 3 meters straight ahead of where we started, facing forward

        new Pose2d(8.5, flipped*1, new Rotation2d(Units.degreesToRadians(90))),

            // Pass config

        Constants.AutoConstants.config);

        Trajectory removeNote2 = TrajectoryGenerator.generateTrajectory(

        // Start at the origin facing the +X direction

        new Pose2d(8.5, flipped*1, new Rotation2d(Units.degreesToRadians(90))),

        // Pass through these two interior waypoints, making an 's' curve path

        List.of(new Translation2d(8.5,  flipped*0.5)),

        // End 3 meters straight ahead of where we started, facing forward

        new Pose2d(8.5, flipped*1, new Rotation2d(Units.degreesToRadians(80))),

            // Pass config

        Constants.AutoConstants.config);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotatePivotGround(infeed),


       new ParallelCommandGroup(new RamseteCommand(

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

            drivebase), new Intake(infeed)).until(infeed::getLimitSwitchReverse),

            new StopAll(infeed, shooter),

            new RotatePivotShooter(infeed),

            new WaitCommand(.5),
            
            new AutoShoot(infeed, shooter),

            new WaitCommand(.2),

            new RotatePivotGround(infeed),


            new ParallelCommandGroup(new RamseteCommand(

            removeNote1,

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

            drivebase), new Intake(infeed)).until(infeed::getLimitSwitchReverse),

            new StopAll(infeed, shooter),

            new RotatePivotShooter(infeed),

            new WaitCommand(.4),

            new AutoAmpShot(shooter, infeed),

            new WaitCommand(.2),

            new RotatePivotGround(infeed),

             new ParallelCommandGroup(new RamseteCommand(

            removeNote2,

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

            drivebase), new Intake(infeed)).until(infeed::getLimitSwitchReverse),

            new StopAll(infeed, shooter),

            new RotatePivotShooter(infeed),

            new WaitCommand(.4),

            new AutoAmpShot(shooter, infeed)
    );

  }
}
