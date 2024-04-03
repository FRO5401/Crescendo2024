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
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.ShiftGear;
import frc.robot.Commands.StopAll;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class SideTwoPiece extends SequentialCommandGroup {
  Drivebase drivebase;
  Infeed infeed;
  Shooter shooter;

  Trajectory leaveStart = TrajectoryGenerator.generateTrajectory(

      new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),

      List.of(new Translation2d(.3, 0)),

      new Pose2d(1, 0, new Rotation2d(Units.degreesToRadians(0))),

      Constants.AutoConstants.config.setReversed(false));

      

      Trajectory shootNote1 = TrajectoryGenerator.generateTrajectory(

      new Pose2d(new Translation2d(1, 0), new Rotation2d(0)),

      List.of(new Translation2d(1, -0.1)),

      new Pose2d(1, -.65, new Rotation2d(Units.degreesToRadians(32))),

      Constants.AutoConstants.config.setReversed(true));



    Trajectory getNote2 = TrajectoryGenerator.generateTrajectory(

      new Pose2d(new Translation2d(.4, -.4), new Rotation2d(Units.degreesToRadians(44))),

      List.of(new Translation2d(1, 1)),

      new Pose2d(7.64, 3.5, new Rotation2d(Units.degreesToRadians(0))),

      Constants.AutoConstants.config.setReversed(false));
      


  /** Creates a new SideTwoPiece. */
  public SideTwoPiece(Drivebase mDrivebase, Infeed mInfeed, Shooter mShooter) {
    drivebase = mDrivebase;
    infeed = mInfeed;
    shooter = mShooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


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

        drivebase)).until(infeed::getLimitSwitchReverse).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),

       new RamseteCommand(

        shootNote1,

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

        drivebase),

      new AutoShoot(infeed, shooter),

      new WaitCommand(.2),

      new StopAll(infeed, shooter),

      new ParallelCommandGroup(new RamseteCommand(

        getNote2,

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

        drivebase), new SequentialCommandGroup(new Intake(infeed),new RotatePivotGround(infeed)).until(infeed::getLimitSwitchReverse).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))


        );
  }
}
