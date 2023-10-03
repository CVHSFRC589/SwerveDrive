// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    m_angleMotor = new CANSparkMax(Constants.ArmConstants.kAngleMotorCanID, MotorType.kBrushed);
    m_spinMotorFront = new CANSparkMax(Constants.ArmConstants.kSpinMotorFrontID, MotorType.kBrushed);
    m_spinMotorBack = new CANSparkMax(Constants.ArmConstants.kSpinMotorBackID, MotorType.kBrushed);
    m_angleEncoder.setPositionConversionFactor(Constants.ArmConstants.kArmGearRatio);
    //Set Encoders
    m_angleMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_angleEncoder = m_angleMotor.getEncoder();
    m_spinEncoderFront = m_spinMotorFront.getEncoder();
    m_spinEncoderBack = m_spinMotorBack.getEncoder();

    resetEncoders();
    m_spinMotorBack.follow(m_spinMotorFront);
    m_spinMotorBack.setInverted(true);
  }

  public void moveArmUp(double speed) {
    if(getCurrentAngle() < Constants.ArmConstants.kMaxArmAngle){
      m_angleMotor.set(speed);
    }
  }

  public void moveArmDown(double speed){
    if(getCurrentAngle() > 0){
      m_angleMotor.set(speed);
    }
  }

  public void stopArm(){
    m_angleMotor.set(0);
  }



  public void resetEncoders() {
    m_angleEncoder.setPosition(0);
    m_spinEncoderFront.setPosition(0);
    m_spinEncoderBack.setPosition(0);
  }

  public void spin(boolean direction) {
    if(direction){
      m_spinMotorFront.set(Constants.ArmConstants.kSpiningSpeed);
    }
    else{
      m_spinMotorFront.set(-Constants.ArmConstants.kSpiningSpeed);
    }
  }

  public void stopSpin(){
    m_spinMotorFront.set(0);
    m_spinMotorBack.set(0);
  }


  public double getCurrentAngle() {
    return m_angleEncoder.getPosition()*Constants.ArmConstants.ArmScaleEncoder;
  }

  public boolean isRaised() {
    return(getCurrentAngle() > Constants.ArmConstants.kMaxArmAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getCurrentAngle());
    SmartDashboard.putBoolean("Is Arm Raised", isRaised());

  }
}
