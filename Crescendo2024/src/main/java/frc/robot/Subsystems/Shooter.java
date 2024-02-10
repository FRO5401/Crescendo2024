// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//WPI Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//File Imports
import frc.robot.Constants;

//Unused Imports that might be used in future
//import edu.wpi.first.math.controller.PIDController;

public class Shooter extends SubsystemBase {
  //two motor on the shooter
  private CANSparkMax leadMotor;
  private CANSparkMax followMotor;

  //Creates encoder on Shooter
  private RelativeEncoder encoder;

  //Creates PID controller for shooter
  private SparkPIDController pidController;

  /** Creates a new Shooter. */
  public Shooter() {
    //Initalizes CANSparkMax Motors
    leadMotor = new CANSparkMax(Constants.ShooterConstants.LEAD_ID, MotorType.kBrushless);
    followMotor = new CANSparkMax(Constants.ShooterConstants.FOLLOWER_ID, MotorType.kBrushless);

    //get encoder from leadmotor
    encoder = leadMotor.getEncoder();

    //LeadMotor inverted so motors can shoot out note
    leadMotor.setInverted(true);

    //the true is used to set it inverted to the lead motor
    followMotor.follow(leadMotor, true);

    //Defines pidController
    pidController = leadMotor.getPIDController();

    //Sets PID values
    pidController.setFF(Constants.ShooterConstants.kF);
    pidController.setP(Constants.ShooterConstants.kP);
    pidController.setI(Constants.ShooterConstants.kI);
    pidController.setD(Constants.ShooterConstants.kD);

  }
  //Sets velocity of both shooter motors
  public void setVelocity(double velocity){
    pidController.setReference(velocity, ControlType.kVelocity);

  }
  //Gets velocity of both shooter motors
  public double getVelocity(){
    return encoder.getVelocity();
  }
  //Stops shooter motors
  public void stop(){
    leadMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Displays shooter velocity to smart dashboard
    SmartDashboard.putNumber("Velocity", getVelocity()); 
  }
}
