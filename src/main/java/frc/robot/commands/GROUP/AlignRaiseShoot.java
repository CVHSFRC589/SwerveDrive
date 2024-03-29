// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GROUP;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ARM.RaiseArm;
import frc.robot.commands.ARM.ShootIntake;
import frc.robot.commands.DRIVE.AutoAlign;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SpinnersSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignRaiseShoot extends SequentialCommandGroup {
  /** Creates a new AlignRaiseShoot. */
  public AlignRaiseShoot(DriveSubsystem drive, ArmSubsystem arm, SpinnersSubsystem spin) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoAlign(drive),
      new RaiseArm(arm, Constants.ArmSpinConstants.kRaisingSpeed),
      new ShootIntake(spin)
    );
  }
}
