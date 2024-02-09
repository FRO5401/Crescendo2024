// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

// Rev imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

// WPI imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Constant imports
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  // Adds CANSparkMax
    private CANSparkMax leftClimberMotor;
    private CANSparkMax rightClimberMotor;

  // Adds Encoder
    private RelativeEncoder leftClimberEncoder;
    private RelativeEncoder rightClimberEncoder;

  public Climber() {
    // Init CANSparkMotors
    leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.LEFTCLIMBER_ID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.RIGHTCLIMBER_ID, MotorType.kBrushless);

    // Sets up climber encoders
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);

    // Resets Motors, factory defaults
    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    // Allows an idle stage
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    // Makes right climber motor inverted
    leftClimberMotor.setInverted(false);
    rightClimberMotor.setInverted(true);
  }

// Get left climber motor position
  public double getLeftPosition() {
    return leftClimberEncoder.getPosition();
  }
  
// Get right climber motor position
  public double getRightPosition() {
    return rightClimberEncoder.getPosition();
  }

// Get left climber motor velocity
  public double getLeftVelocity() {
    return leftClimberEncoder.getVelocity();
  }

// Get left climber motor velocity
  public double getRightVelocity() {
    return rightClimberEncoder.getVelocity();
  }

// Left up
  public void leftClimbUp() {
    if (leftClimberEncoder.getPosition() > Constants.ClimberConstants.climberEncoderMax) {
      leftClimberMotor.set(0);
    } else {
      leftClimberMotor.set(Constants.ClimberConstants.climberUpSpeed);
    }
  }

// Right up
  public void rightClimbUp() {
    if (rightClimberEncoder.getPosition() > Constants.ClimberConstants.climberEncoderMax) {
      rightClimberMotor.set(0);
    } else {
      rightClimberMotor.set(Constants.ClimberConstants.climberUpSpeed);
    }
  }

// Left down
  public void leftClimbDown() {
    if (leftClimberEncoder.getPosition() < Constants.ClimberConstants.climberEncoderMin) {
      leftClimberMotor.set(0);
    } else {
      leftClimberMotor.set(Constants.ClimberConstants.climberDownSpeed);
    }
  }


// Right down
  public void rightClimbDown() {
    if (rightClimberEncoder.getPosition() < Constants.ClimberConstants.climberEncoderMin) {
      rightClimberMotor.set(0);
    } else {
      rightClimberMotor.set(Constants.ClimberConstants.climberDownSpeed);
    }
  }


  // Reset left encoder
  public void resetLeftEncoder() {
    leftClimberEncoder.setPosition(0);
  }

  // Reset right encoder
  public void resetRightEncoder() {
    rightClimberEncoder.setPosition(0);
  }

  // Reset both encoder
  public void resetEncoders() {
    leftClimberEncoder.setPosition(0);
    rightClimberEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Displays climber encoder values on SmartDashBoard
    SmartDashboard.putNumber("Left Climber Motor-Encoder Value: ", getLeftPosition());
    SmartDashboard.putNumber("Right Climber Motor-Encoder Value: ", getRightPosition());
  }
}