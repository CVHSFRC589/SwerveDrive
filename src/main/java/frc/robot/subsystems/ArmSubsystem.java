// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_angleMotor;
  private CANSparkMax m_spinMotorFront;
  private CANSparkMax m_spinMotorBack;
  private RelativeEncoder m_angleEncoder;
  private RelativeEncoder m_spinEncoderFront;
  private RelativeEncoder m_spinEncoderBack;
  
  public ArmSubsystem() {
    //Set Motors
    m_angleMotor = new CANSparkMax(Constants.ArmConstants.kAngleMotorCanId, MotorType.kBrushed);
    m_spinMotorFront = new CANSparkMax(Constants.ArmConstants.kSpinMotorFront, MotorType.kBrushed);
    m_spinMotorBack = new CANSparkMax(Constants.ArmConstants.kSpinMotorBack, MotorType.kBrushed);

    //Set Encoders
    m_angleEncoder = m_angleMotor.getEncoder();
    m_spinEncoderFront = m_spinMotorFront.getEncoder();
    m_spinEncoderBack = m_spinMotorBack.getEncoder();

    resetEncoders();
  }

  public void moveArm(double speed) {

  }


  public void resetEncoders() {
    m_angleEncoder.setPosition(0);
    m_spinEncoderFront.setPosition(0);
    m_spinEncoderBack.setPosition(0);
  }

  public void grab() {

  }

  public void shoot() {

  }

  public double getCurrentAngle() {
    return 0;
    //tbd
  }

  public boolean isRaised() {
    return true;
    // tbd
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
