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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Commands.Expel;
import frc.robot.Commands.Intake;
import frc.robot.Commands.RotatePivotGround;
import frc.robot.Commands.RotatePivotShooter;
import frc.robot.Commands.SpeakerShot;
import frc.robot.Commands.StopAll;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Infeed;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceAuto extends SequentialCommandGroup {
  Infeed infeed;
  Shooter shooter;
  Drivebase drivebase;

  Trajectory leaveStart;
  Trajectory backToSpeaker;
  Trajectory getNote1;
  Trajectory getNote2;

  int flipped;
  /** Creates a new ThreePieceAuto. */
  public FourPieceAuto(  Infeed m_infeed, Shooter m_shooter, Drivebase m_drivebase) {
     infeed = m_infeed;
    shooter = m_shooter;
    drivebase = m_drivebase;

    if(DriverStation.getAlliance().get() == Alliance.Blue){
      flipped = -1;
    } else {
      flipped = 1;
    }

    leaveStart = TrajectoryGenerator.generateTrajectory(

      drivebase.getPose(),

      List.of(new Translation2d(.7, 0)),

      new Pose2d(1.3, 0, new Rotation2d(0)),

      Constants.AutoConstants.config.setReversed(false));


      backToSpeaker = TrajectoryGenerator.generateTrajectory(

      new Pose2d(1.3, 0, new Rotation2d(0)),

      List.of(new Translation2d(.6, 0)),


      new Pose2d(0, 0, new Rotation2d(0)),

      Constants.AutoConstants.config.setReversed(true));
    // Add your commands in the addCommands() call, e.g.

    getNote1 = TrajectoryGenerator.generateTrajectory(

      drivebase.getPose(),

      List.of(new Translation2d(.25, flipped*-.45)),

      new Pose2d(1.2, flipped*-1.35, new Rotation2d(0)),

      Constants.AutoConstants.config.setReversed(false));

    getNote2 = TrajectoryGenerator.generateTrajectory(

      drivebase.getPose(),

      List.of(new Translation2d(.25, flipped*.45)),

      new Pose2d(1.2, flipped*1.35, new Rotation2d(0)),

      Constants.AutoConstants.config.setReversed(false));

    addRequirements(drivebase,infeed,shooter);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


      new AutoShoot(infeed, shooter),

      new WaitCommand(.35),

      new StopAll(infeed, shooter),

      new RotatePivotGround(infeed),


          new ParallelCommandGroup( new RamseteCommand(

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

           new ParallelCommandGroup( new RamseteCommand(

            backToSpeaker,

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

            drivebase), new RotatePivotShooter(infeed), new SpeakerShot(shooter)),

            new Expel(infeed),

        new WaitCommand(.2),

        new StopAll(infeed, shooter),

        new RotatePivotGround(infeed),

        new ParallelCommandGroup(new RamseteCommand(

            getNote1,

            drivebase::getPose,

            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),

            new SimpleMotorFeedforward(

                Constants.AutoConstants.ksVolts,

                Constants.AutoConstants.kvVoltSecondsPerMeter,

                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),

            Constants.AutoConstants.kDriveKinematics,

            drivebase::getWheelSpeeds,

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            // RamseteCommand passes volts to the callback

            drivebase::tankDriveVolts,

            drivebase), new Intake(infeed)).until(infeed::getLimitSwitchReverse),

            new StopAll(infeed, shooter),

            new ParallelCommandGroup( new RamseteCommand(

            backToSpeaker,

            drivebase::getPose,

            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),

            new SimpleMotorFeedforward(

                Constants.AutoConstants.ksVolts,

                Constants.AutoConstants.kvVoltSecondsPerMeter,

                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),

            Constants.AutoConstants.kDriveKinematics,

            drivebase::getWheelSpeeds,

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            // RamseteCommand passes volts to the callback

            drivebase::tankDriveVolts,

            drivebase), new RotatePivotShooter(infeed), new SpeakerShot(shooter)),
            
            new Expel(infeed),

            new WaitCommand(.2),

            new StopAll(infeed, shooter),

            new RotatePivotGround(infeed),

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

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            new PIDController(Constants.AutoConstants.kPDriveVel  , 0, 0),

            // RamseteCommand passes volts to the callback

            drivebase::tankDriveVolts,

            drivebase), new Intake(infeed)).until(infeed::getLimitSwitchReverse),

            new StopAll(infeed, shooter),

            new ParallelCommandGroup( new RamseteCommand(

            backToSpeaker,

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

            drivebase), new RotatePivotShooter(infeed), new SpeakerShot(shooter)),

            
            new Expel(infeed)

    );
  }
}
