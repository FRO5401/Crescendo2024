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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Trajectorys;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Drivebase;

public class TestAuto2  extends Command {
  Trajectory m_exampleTrajectory;

  Drivebase drivebase;

  /** Creates a new TestAuto. */
  public TestAuto2(Drivebase m_Drivebase) {
    drivebase = m_Drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Auto Position", drivebase.getPose().getX());

        
    m_exampleTrajectory = TrajectoryGenerator.generateTrajectory(

    // Start at the origin facing the +X direction

    drivebase.getPose(),

    // Pass through these two interior waypoints, making an 's' curve path

    List.of(new Translation2d(4, 0), new Translation2d(5, -1)),

    // End 3 meters straight ahead of where we started, facing forward

    new Pose2d(6, -1, new Rotation2d(0)),

        // Pass config

    Constants.AutoConstants.config);


        CommandScheduler.getInstance().schedule(new RamseteCommand(

        Trajectorys.makeTrajectory(drivebase),

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

        drivebase));

        drivebase.updateOdometry();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
