// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_angleMotor;
  private AbsoluteEncoder m_angleEncoder;

  public ArmSubsystem() {
    //Set Motors
    m_angleMotor = new CANSparkMax(Constants.ArmSpinConstants.kAngleMotorCanID, MotorType.kBrushed);
    m_angleEncoder.setPositionConversionFactor(Constants.ArmSpinConstants.kArmGearRatio);
    //Set Encoders
    m_angleMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_angleEncoder = m_angleMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  }

  public void moveArmUp(double speed) {
    if (getCurrentAngle() < Constants.ArmSpinConstants.kMaxArmAngle) {
      m_angleMotor.set(speed);
    }
  }

  public void moveArmDown(double speed) {
    if (getCurrentAngle() > 0) {
      m_angleMotor.set(speed);
    }
  }

  public void stopArm() {
    m_angleMotor.set(0);
  }

  public double getCurrentAngle() {
    return m_angleEncoder.getPosition() * Constants.ArmSpinConstants.ArmScaleEncoder;
  }

  public boolean isRaised() {
    return (getCurrentAngle() > Constants.ArmSpinConstants.kMaxArmAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Angle", getCurrentAngle());
    SmartDashboard.putBoolean("Is Arm Raised", isRaised());

  }
}
