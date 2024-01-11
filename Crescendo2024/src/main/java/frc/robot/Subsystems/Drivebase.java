// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.Collection;

/** Add your docs here. */
public class Drivebase {
    private CANSparkMax leftDrive1;
    private CANSparkMax leftDrive2;
    private CANSparkMax leftDrive3;
  
    private CANSparkMax rightDrive1;
    private CANSparkMax rightDrive2;
    private CANSparkMax rightDrive3;
  
    private SparkPIDController leftDrive1PidController;
    private SparkPIDController leftDrive2PidController;
    private SparkPIDController leftDrive3PidController;
  
    private SparkPIDController rightDrive1PidController;
    private SparkPIDController rightDrive2PidController;
    private SparkPIDController rightDrive3PidController;
  
    DigitalInput minSwitch, maxSwitch;
    

    private MotorController leftDrives;
  private MotorController rightDrives;

  private DifferentialDrive allDrive;

  private DifferentialDriveOdometry odometry;

  private int motorCurrent = 24;

  private boolean compressorState = false;

  private double kP = 1.92;
  private double kI = 0;
  private double kD = 1;

  private double kEncoder = 0.01;
  double percentDifference;

  private Solenoid gearShifter;

  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  public Compressor compressor;
  private PowerDistribution pdp;
  Collection<CANSparkMax> drivebaseMotors;
  SparkPIDController pidRotateMotorLeft;
  //PhotonTrackedTarget target;
 }


