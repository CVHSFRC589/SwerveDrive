// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DRIVE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  private DriveSubsystem m_drive;
  public AutoAlign(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.drive(0, 0, 0, true, false);
    if(m_drive.getGyroYaw()>177.5&&m_drive.getGyroYaw()>-177.5){
      end(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentyaw = m_drive.getGyroYaw();
    if(currentyaw<175.5){
      System.out.print("1");
      m_drive.drive(0, 0, .4, true, false);
    }
    else if(currentyaw/-1!=currentyaw){
      System.out.print("2");  
      m_drive.drive(0, 0, -.4, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true, false);
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drive.getGyroYaw()>177.5&&m_drive.getGyroYaw()>-177.5);
  }
}
