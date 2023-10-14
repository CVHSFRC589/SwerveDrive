// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AUTO;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DRIVE.AutoBalance;
import frc.robot.commands.DRIVE.ResetOdom;
import frc.robot.subsystems.DriveSubsystem;
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
                                5,
                                7)
                                .setKinematics(DriveConstants.kDriveKinematics);

                Trajectory p12 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0.01)),
                                List.of(new Translation2d(1, -1)),
                                new Pose2d(2, 0, new Rotation2d(3.14)),
                                tConfig);
                
                


                                ////////////////////
                Trajectory p1 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(3.14)),
                                List.of(),
                                new Pose2d(0, 2, new Rotation2d(1.57)),
                                tConfig);

                Trajectory p2 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 1.5, new Rotation2d(1.57)),
                                List.of(),
                                new Pose2d(-4.8, 2, new Rotation2d(0)),
                                tConfig);

                Trajectory p3 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(-4.8, 2, new Rotation2d(0)),
                                List.of(),
                                new Pose2d(-4.1, 2, new Rotation2d(0)),
                                tConfig);

                Trajectory p4 = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(-4.1, -.5, new Rotation2d(0)),
                                List.of(),
                                new Pose2d(-2.6, -.5, new Rotation2d(0)),
                                tConfig);

                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(0, 1.5),
                // new Translation2d(-4.8, 1.5),
                // new Translation2d(-4.1, 2),
                // new Translation2d(-2.6, 0)
                // ),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(-2.6, 0, new Rotation2d(0)),
                // tConfig);

                SwerveControllerCommandMaker cmdMaker = new SwerveControllerCommandMaker(testThetaController,
                                xPIDController,
                                yPIDController, drive);
                SwerveControllerCommand cmd1 = cmdMaker.makeCommand(p1);
                SwerveControllerCommand cmd2 = cmdMaker.makeCommand(p2);
                SwerveControllerCommand cmd3 = cmdMaker.makeCommand(p3);
                SwerveControllerCommand cmd4 = cmdMaker.makeCommand(p4);
                SwerveControllerCommand cmd5 = cmdMaker.makeCommand(p12);
                // SwerveControllerCommand cmd5 = cmdMaker.makeCommand(exampleTrajectory);

                drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0.01)));
                // drive.setGyro(180);
                addCommands(
                                new ResetOdom(drive, 0.01),
                                cmd5
                                // // new RaiseArm(arm)
                                // // new ThrowArm(arm),
                                // cmd1,
                                // new WaitCommand(2),
                                // cmd2,
                                // // new GrabArm(arm),
                                // new WaitCommand(2),
                                // cmd3,
                                // new WaitCommand(2),
                                // cmd4,
                                // new WaitCommand(2),
                                // // cmd5,
                                // // new WaitCommand(4),
                                // new AutoBalance(drive)

                );
        }
}
