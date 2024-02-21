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

    public static class PhotonConstants{
        public static final double CAMERA_ANGLE = 25;
        public static final double CAMERA_HEIGHT = 24.5;
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
        public static final double KP = 1.2;
        public static final double KI = 0;
        public static final double KD = 0; 
    
        // Sensitivity Constants
        public static final double PERCISION_SENSITIVITY = 0.4;
        public static final double DEFAULT_SENSITIVTY = 1.0;
    
        // Define stopping
        public static final double POWER_STOP = 0;
    
        // Inverse Direction of Drive Motor
        public static final int INVERSE_DIRECTION = -1;
    
        // Straight Direction
        public static final double STRAIGHT_DIRECTION = 1.0;
        public static final double ANGULAR_KP = 0.03;
        public static final double ANGULAR_KI = 0;
        public static final double ANGULAR_KD = 0; 
    }
    
    //Infeed Constants
    public static class InfeedConstants{
        //Id of infeeds CANSparkMax motors
        public static final int INTAKE1_ID = 6;
        public static final int PIVOT_ID = 5;

        //PID values
        public static final double pivotP = .1;
        public static final double pivotI = 0.000005;
        public static final double pivotD = 9.5;
        public static final double pivotILimit = .01;

        //set pointa
        public static final double IN_POSITION = -.5;
        public static final double SAFE_POSITION = -2;

        public static final double OUT_POSITION = -14.2;
        public static final double AIR_POSITION = -6.5;

        /* Infeed speed constants */

        //Take in note speed
        public static final double INTAKE_SPEED = 0.6;
        //Remove note from infeed speed
        public static final double EXPEL_SPEED = -0.43;
        //Move Pivot to floor speed
        public static final double PIVOT_TO_GROUND_SPEED = -0.35;
        //Move Pivot to shooter speed
        public static final double PIVOT_TO_SHOOTER_SPEED = 0.35;
        //Fix Pivot position if overshot speed
        public static final double OVERSHOOT_FIX_SPEED = -0.20;
    }


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
        public static final double pivotP = 0;
        public static final double pivotI = 0;
        public static final double pivotD = 0;

    

    public static class ClimberConstants {
        //IDs of climber CANSparkMax motors
        public static final int LEFTCLIMBER_ID = 10;
        public static final int RIGHTCLIMBER_ID = 9;

        // Encoder ranges
        public static final int climberEncoderMax = 25;
        public static final int climberEncoderMin = 0;

        // Climb motor speed
        public static final double climberUpSpeed = 1;
        public static final double climberDownSpeed = -1;
        public static final double CONTROLLER_DEADZONE = .05;

    }
}

