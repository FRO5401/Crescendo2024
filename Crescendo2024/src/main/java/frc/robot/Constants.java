// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final int LEFT_DRIVE1_ID = 3;
        public static final int LEFT_DRIVE2_ID = 4;
        public static final int RIGHT_DRIVE1_ID = 1;
        public static final int RIGHT_DRIVE2_ID = 2;
    
        // Maxium amount of AMPS allowed to the motors
        public static final int CURRENT_LIMIT = 30;
    
        // PID Constants
        public static final int KP = 0;
        public static final int KI = 0;
        public static final int KD = 0; 
    
        // Sensitivity Constants
        public static final double PERCISION_SENSITIVITY = 0.4;
        public static final double DEFAULT_SENSITIVTY = 1.0;
    
        // Define stopping
        public static final double POWER_STOP = 0;
    
        // Inverse Direction of Drive Motor
        public static final int INVERSE_DIRECTION = -1;
    
        // Straight Direction
        public static final double STRAIGHT_DIRECTION = 1.0; 
    }
    
    //Infeed Constants
    public static class InfeedConstants{
        //Id of infeeds CANSparkMax motors
        public static final int INTAKE1_ID = 6;
        public static final int PIVOT_ID = 5;

        //PID values
        public static final double pivotP = .1;
        public static final double pivotI = 0;
        public static final double pivotD = 10;

        //set pointa
        public static final double IN_POSITION = -1;
        public static final double OUT_POSITION = -13.6;
        public static final double AIR_POSITION = -7;

        /* Infeed speed constants */

        //Take in note speed
        public static final double INTAKE_SPEED = 0.5;
        //Remove note from infeed speed
        public static final double EXPEL_SPEED = -0.5;
        //Move Pivot to floor speed
        public static final double PIVOT_TO_GROUND_SPEED = -0.35;
        //Move Pivot to shooter speed
        public static final double PIVOT_TO_SHOOTER_SPEED = 0.35;
        //Fix Pivot position if overshot speed
        public static final double OVERSHOOT_FIX_SPEED = -0.20;
    public final class ShooterConstants{
        public static final int LEAD_ID = 7;
        public static final int FOLLOWER_ID = 8;

        public static final double kF = .00017;
        public static final double kP = .00015;
        public static final double kI = 0;
        public static final double kD = .00000;

        public final class SpeedConstants{
            public static final double SHOOTER_RPM = 5500;
            public static final double AMP_RPM = 2500;
            public static final double TRAP_RPM = 4000;
        }
    }

  }