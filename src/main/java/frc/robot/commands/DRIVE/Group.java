// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DRIVE;

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
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveControllerCommandMaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Group extends SequentialCommandGroup {
  /** Creates a new Group. */
  public Group(DriveSubsystem drive) {
    ProfiledPIDController testThetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // testThetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);

    TrajectoryConfig testConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    // Add your commands in the addCommands() call, e.g.
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(
        // 0,-1)),
        // End 3 meters straight ahead of where we started,% facing forward
        new Pose2d(0, -1, new Rotation2d()),
        testConfig);

    Trajectory testTrajectory2 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, -1, new Rotation2d(0)),
        List.of(),
        // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(
        // 0,-1)),
        // End 3 meters straight ahead of where we started,% facing forward
        new Pose2d(0, 0, new Rotation2d()),
        testConfig);

    SwerveControllerCommandMaker cmdMaker = new SwerveControllerCommandMaker(testThetaController, xPIDController,
        yPIDController, drive);
    Pose2d testPose2d = new Pose2d(1, -1, new Rotation2d(1));
    SwerveControllerCommand cmd1 = cmdMaker.makeCommand(testTrajectory);
    SwerveControllerCommand cmd4 = cmdMaker.makeCommand(testTrajectory2);
    // SwerveControllerCommand cmd2 = cmdMaker.makeCommandPose(testPose2d,
    // testConfig);
    // Sw erveControllerCommand cmd3 = cmdMaker.makeCommandPose(new Pose2d(0, 0, new
    // Rotation2d(0)), testConfig);
    drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    addCommands(
        cmd1,
        new WaitCommand(2),
        cmd4
    // cmd2,
    // cmd3,
    // new WaitCommand(3)
    // cmd2
    // //new DriveForward(0.2, .5, drive, 0),
    // new DriveSideWays(.2, .96, drive, 0),
    // //new DriveRotate(.2,.58, drive,0)
    // // new DriveSideWays( .2, .96, drive, 0),
    // new DriveForward(.2, .71, drive, 0),
    // new DriveRotate(-.2,.29, drive, 0),
    // new DriveForward(.2, 2.14, drive, 0)

    );
  }
}
