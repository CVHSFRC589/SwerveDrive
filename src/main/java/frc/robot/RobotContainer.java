// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DRIVE.AutoBalance;
import frc.robot.commands.DRIVE.Group;
import frc.robot.commands.DRIVE.setX;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveControllerCommandMaker;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        // private final Gyro m_Gyro = new Gyro();
        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(2),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true),
                                                m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                new JoystickButton(m_driverController, 3)
                                .toggleOnTrue(new AutoBalance(m_robotDrive));
                new JoystickButton(m_driverController, 2)
                                .toggleOnTrue(new setX(m_robotDrive));

                new JoystickButton(m_driverController, Button.kR1.value)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(0, -1)),
                                // End 3 meters straight ahead of where we started,% facing forward
                                new Pose2d(0, 0, new Rotation2d()),
                                config);
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(0, 0)),//, new Translation2d(2, 0), new
                // Translation2d(0, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(0 , 0, new Rotation2d(Math.PI)),
                // config);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // true, false));
                return new Group(m_robotDrive);
                // return cmd2;
        }

        /////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////

        // public SwerveControllerCommandMaker(
        // ProfiledPIDController thetaController, PIDController xPIDController,
        // PIDController yPIDController,
        // DriveSubsystem robotDrive) {

        TrajectoryConfig testConfig = new TrajectoryConfig(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics);

        ProfiledPIDController testThetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // testThetaController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);

        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        // Pass through these two interior waypoints, making an 's' curve path
                        // List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(
                        // 0,-1)),
                        // End 3 meters straight ahead of where we started,% facing forward
                        new Pose2d(.5, 0, new Rotation2d()),
                        testConfig);

        Trajectory testTrajectory2 = TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(),
                        // Pass through these two interior waypoints, making an 's' curve path
                        // List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(
                        // 0,-1)),
                        // End 3 meters straight ahead of where we started,% facing forward
                        new Pose2d(1, 0, new Rotation2d()),
                        testConfig);

        SwerveControllerCommandMaker cmdMaker = new SwerveControllerCommandMaker(testThetaController, xPIDController,
                        yPIDController, m_robotDrive);
        Pose2d testPose2d = new Pose2d(1, -1, new Rotation2d(1));
        SwerveControllerCommand cmd1 = cmdMaker.makeCommand(testTrajectory);
        SwerveControllerCommand cmd2 = cmdMaker.makeCommandPose(testPose2d, testConfig);

}
