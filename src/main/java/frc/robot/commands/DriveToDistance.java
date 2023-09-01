// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistance extends CommandBase {
  private DriveSubsystem m_drive;
  private double m_x;
  private double m_y;
  private double m_orgX;
  private double m_originalY;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(DriveSubsystem drive, double x, double y) {
    m_drive = drive;
    m_x = x;
    // m_y = y;
    m_orgX = m_drive.getEncoderMeters();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
