// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinnersSubsystem extends SubsystemBase {
  /** Creates a new SpinnersSubsystem. */

  private CANSparkMax m_spinMotorFront;
  private CANSparkMax m_spinMotorBack;
  private RelativeEncoder m_spinEncoderFront;
  private RelativeEncoder m_spinEncoderBack;

  public SpinnersSubsystem() {

    m_spinMotorFront = new CANSparkMax(Constants.ArmSpinConstants.kSpinMotorFrontID, MotorType.kBrushed);
    m_spinMotorBack = new CANSparkMax(Constants.ArmSpinConstants.kSpinMotorBackID, MotorType.kBrushed);

    m_spinEncoderFront = m_spinMotorFront.getEncoder();
    m_spinEncoderBack = m_spinMotorBack.getEncoder();

    m_spinMotorBack.follow(m_spinMotorFront);
    m_spinMotorBack.setInverted(true);
  }

  public void resetEncoders() {
    m_spinEncoderFront.setPosition(0);
    m_spinEncoderBack.setPosition(0);
  }

  public void spin(boolean direction) {
    if(direction){
      m_spinMotorFront.set(Constants.ArmSpinConstants.kSpiningSpeed);
    }
    else{
      m_spinMotorFront.set(-Constants.ArmSpinConstants.kSpiningSpeed);
    }
  }

  public void stopSpin(){
    m_spinMotorFront.set(0);
    m_spinMotorBack.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
