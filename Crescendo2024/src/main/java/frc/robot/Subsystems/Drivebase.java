// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.Collection;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  PhotonCamera camera;
  PhotonCamera camera2;

  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  

  private SparkMaxPIDController leftDrive1PidController;
  private SparkMaxPIDController leftDrive2PidController;
 
  private SparkMaxPIDController rightDrive1PidController;
  private SparkMaxPIDController rightDrive2PidController;
  

  DigitalInput minSwitch, maxSwitch;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive allDrive;
  private DifferentialDriveOdometry odometry;
  private int motorCurrent = 24;

  private boolean compressorState = false;

    // PID stuff
  // private int loopIndex, slotIndex;
  private double kP = 1.92;
  private double kI = 0;
  private double kD = 1;
   // Solenoid
  private Solenoid gearShifter;





