// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmSpinConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private CANSparkMax m_angleMotor;
  private AbsoluteEncoder m_angleEncoder;
  private SparkMaxPIDController m_PIDController;

  public ArmSubsystem() {
    // Set Motors
    m_angleMotor = new CANSparkMax(Constants.ArmSpinConstants.kAngleMotorCanID, MotorType.kBrushless);
    // Set Encoders
    m_angleMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    m_angleEncoder = m_angleMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_angleEncoder.setPositionConversionFactor(Constants.ArmSpinConstants.kArmGearRatio);



    m_PIDController = m_angleMotor.getPIDController();

    m_PIDController.setSmartMotionMaxVelocity(ArmSpinConstants.kMaxArmAngle, 0);
    m_PIDController.setSmartMotionMinOutputVelocity(ArmSpinConstants.kArmMinRPM, 0);
    m_PIDController.setSmartMotionMaxAccel(ArmSpinConstants.kArmMaxAccel, 0);
    m_PIDController.setSmartMotionAllowedClosedLoopError(ArmSpinConstants.kArmAllowedErr, 0);
    m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_PIDController.setP(ArmSpinConstants.kP, 0);
    m_PIDController.setI(ArmSpinConstants.kI, 0);
    m_PIDController.setD(ArmSpinConstants.kD, 0);
    m_PIDController.setIZone(ArmSpinConstants.kI, 0);
    m_PIDController.setFF(ArmSpinConstants.kFF, 0);


  }

  public void moveArm(DoubleSupplier speed) {
    // m_angleMotor.set(speed.getAsDouble() * .5);
    m_PIDController.setReference(speed.getAsDouble(), ControlType.kVelocity, 0);
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
    // m_angleMotor.set(0);
    m_PIDController.setReference(m_angleEncoder.getPosition(), ControlType.kVelocity, 0);
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
