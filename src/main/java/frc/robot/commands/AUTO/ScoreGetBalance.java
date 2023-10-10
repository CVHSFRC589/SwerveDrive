// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DRIVE.*;
import frc.robot.commands.ARM.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.utils.SwerveControllerCommandMaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreGetBalance extends SequentialCommandGroup {
  /** Creates a new Group. */
  public ScoreGetBalance(DriveSubsystem drive) { // ,ArmSubsystem arm) {
    ProfiledPIDController testThetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    PIDController xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);

    TrajectoryConfig tConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        5)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory p1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(0, -1, new Rotation2d()),
        tConfig);

    Trajectory p2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, -1, new Rotation2d(0)),
        List.of(),
        new Pose2d(4.6, -1, new Rotation2d(1.8)),
        tConfig);

    Trajectory p3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.6, -1, new Rotation2d(0)),
        List.of(),
        new Pose2d(4.1, 0, new Rotation2d()),
        tConfig);

    Trajectory p4 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(4.1, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(1, 0, new Rotation2d()),
        tConfig);
    // POINTS
    // 0,0 start at scoring positon
    // 0,-1 to back up
    // 4.6, -1
    // 4.1, 0
    // 1, 0

    SwerveControllerCommandMaker cmdMaker = new SwerveControllerCommandMaker(testThetaController, xPIDController,
        yPIDController, drive);
    SwerveControllerCommand cmd1 = cmdMaker.makeCommand(p1);
    SwerveControllerCommand cmd2 = cmdMaker.makeCommand(p2);
    SwerveControllerCommand cmd3 = cmdMaker.makeCommand(p3);
    SwerveControllerCommand cmd4 = cmdMaker.makeCommand(p4);

    drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    drive.setGyro(180);
    addCommands(
        // new RaiseArm(arm)
        // new ThrowArm(arm),
        cmd1,
        new WaitCommand(2),
        cmd2,
        // new GrabArm(arm),
        new WaitCommand(2),
        cmd3,
        new WaitCommand(2),
        cmd4,
        new WaitCommand(2),
        new AutoBalance(drive)

    );
  }
}
