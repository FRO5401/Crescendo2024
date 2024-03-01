// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Drivebase;

/** Add your docs here. */
public class Trajectorys {
    public static Trajectory makeTrajectory(Drivebase drivebase){
        return TrajectoryGenerator.generateTrajectory(

        // Start at the origin facing the +X direction

        drivebase.getPose(),

        // Pass through these two interior waypoints, making an 's' curve path

        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

        // End 3 meters straight ahead of where we started, facing forward

        new Pose2d(3, 0, new Rotation2d(0)),

            // Pass config

        Constants.AutoConstants.config);


    }
}
