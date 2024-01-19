// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
SparkPIDController armPID;
CANSparkMax armMotor;
RelativeEncoder encoder;


  public Arm() {
    //
    armMotor = new CANSparkMax(Constants.ArmConstants.ARM_ID, MotorType.kBrushless);
    armPID = armMotor.getPIDController();
    encoder = armMotor.getEncoder();

    //Reset Arm
    armMotor.restoreFactoryDefaults();
    //Idle mode, Let's mode be
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void armForward(){
    armMotor.set(0.20);
    }
  //Moves arm backward
  public void armBackward(){
    armMotor.set(-0.20);
    }
  public void StopArm(){
    armMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
