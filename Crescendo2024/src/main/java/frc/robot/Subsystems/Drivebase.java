// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Utilities.Tabs.*;

import java.util.Collection;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivebase extends SubsystemBase {

  //Calling Variables (Motors & Encoders)
  private SparkMAXMotorGroup leftDrives = Robot.hardware.leftDriveMotors;
  private SparkMAXMotorGroup rightDrives = Robot.hardware.rightDriveMotors;

  private RelativeEncoder leftDrive1Enc = Robot.hardware.leftDrive1Enc;
  private RelativeEncoder leftDrive2Enc = Robot.hardware.leftDrive2Enc;
  private RelativeEncoder rightDrive1Enc = Robot.hardware.rightDrive1Enc;
  private RelativeEncoder rightDrive2Enc = Robot.hardware.rightDrive2Enc;

  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  private PowerDistribution pdp;
  Collection<CANSparkMax> drivebaseMotors;
  SparkMaxPIDController pidRotateMotorLeft;

  //Variables
  private final AHRS navX;
  private final PIDController turn_PID;
  private DifferentialDriveOdometry odometry;

  public double turn_kP;
  public double turn_kI;
  public double turn_kD;
  public double turn_tolerance;
  public double turn_derivativeTolerance;
  public double turn_error;

  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;

  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive allDrive;

  private SparkMaxPIDController leftDrive1PidController;
  private SparkMaxPIDController leftDrive2PidController;

  private SparkMaxPIDController rightDrive1PidController;
  private SparkMaxPIDController rightDrive2PidController;

  /** Creates a new Drivebase. */
  public Drivebase() {
    leftDrives.setInverted(true);
    rightDrives.setInverted(false);
  }

  public void setPercentOutput(double leftPower, double rightPower) {
    leftDrives.set(leftPower);
    rightDrives.set(rightPower);
    SmartDashboard.putNumber("Left Drive Output ", leftPower);
    SmartDashboard.putNumber("Right Drive Output ", rightPower);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
