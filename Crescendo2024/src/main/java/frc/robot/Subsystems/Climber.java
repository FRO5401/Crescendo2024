// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  CANSparkMax leader;
  CANSparkMax follow;

  RelativeEncoder encoder;

  SparkPIDController pidController;

  Solenoid gear;
  boolean isHighGear = false; 

  /** Creates a new Climber. */
  public Climber() {
    leader = new CANSparkMax(Constants.ClimberConstants.LEADER_ID, MotorType.kBrushless);
    follow = new CANSparkMax(Constants.ClimberConstants.FOLLOW_ID, MotorType.kBrushless);

    encoder = leader.getEncoder();

    encoder.setPosition(0);

    gear = new Solenoid(PneumaticsModuleType.REVPH, 1);

    follow.follow(leader);

    leader.restoreFactoryDefaults();
    follow.restoreFactoryDefaults();

    pidController = leader.getPIDController();

    pidController.setP(Constants.ClimberConstants.kP);
    pidController.setI(Constants.ClimberConstants.kI);
    pidController.setD(Constants.ClimberConstants.kD);
  }

  public void gearShiftHigh(){
    isHighGear = true;
    gear.set(isHighGear);
  }

    public void gearShiftLow(){
    isHighGear = false;
    gear.set(isHighGear);
  }

  public void extendClimber(){
    gearShiftHigh();
    pidController.setReference(Constants.ClimberConstants.FULLY_EXTENDED, ControlType.kPosition);
  }

  public void retractClimber(){
    gearShiftLow();
    pidController.setReference(Constants.ClimberConstants.FULLY_EXTENDED, ControlType.kPosition);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
