// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class ControlConstants{
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        public static final int XBOX_CONTROLLER_OPERATOR =1;
        public static final double AXIS_THRESHOLD = 0.05;
        public static final double SPIN_SENSITIVITY = 0.8;
        
        }
    
    //Controls Constants
    public static class Controls{
        // Defines Driver Controller port number
        public static final int XBOX_CONTROLLER_DRIVER = 0;
        // Defines Driver Operator port number
        public static final int XBOX_CONTROLLER_OPERATOR = 1;
    
        public static final double AXIS_THRESHOLD = 0.05;
        public static final double SPIN_SENSITIVITY = 0.8;
        public static final double DEFAULT_SENSITIVTY = 0.8;
        }
    
    //Drive Motors Constants
    public static class DriveMotors{
        // Defined Motor IDs
        public static final int LEFT_DRIVE1_ID = 1;
        public static final int LEFT_DRIVE2_ID = 3;
        public static final int RIGHT_DRIVE1_ID = 2;
        public static final int RIGHT_DRIVE2_ID = 4;
    
        // Maxium amount of AMPS allowed to the motors
        public static final int CURRENT_LIMIT = 30;
    
        // PID Constants
        public static final int KP = 0;
        public static final int KI = 0;
        public static final int KD = 0; 
    
        // Sensitivity Constants
        public static final double PERCISION_SENSITIVITY = 0.5;
        public static final double DEFAULT_SENSITIVTY = 1.0;
    
        // Define stopping
        public static final double POWER_STOP = 0;
    
        // Inverse Direction of Drive Motor
        public static final double INVERSE_DIRECTION = -1;
    
        // Straight Direction
        public static final double STRAIGHT_DIRECTION = 1.0; 
        
        /* TODO Get actual values */
        /* PathPlanner Constants / Auto Constants */
        //Values need to be changed
        public static final double ksVolts = 0.1219;
        public static final double kvVoltSecondsPerMeter = 3.343;
        public static final double kaVoltSecondsSquaredPerMeter = 1.0356;
        public static final double kPDriveVel = 2.2662;

        public static final double kTrackWidthMeters = Units.inchesToMeters(23);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);
        
        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kGearRatio = 12.6;
        public static final double kWheelRadiusInches = 3;

        public static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));
    }
}