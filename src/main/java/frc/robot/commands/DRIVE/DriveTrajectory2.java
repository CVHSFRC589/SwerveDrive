package frc.robot.commands.DRIVE;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.DriveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class DriveTrajectory2 extends InstantCommand {
//   private final Trajectory m_trajectory;
//   private final DriveSubsystem m_drive;
//   public DriveTrajectory2(Trajectory trajectory, DriveSubsystem drive, double maxspeed, double maxaccel) {
//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(maxspeed, maxaccel);

//     // An example trajectory to follow. All units in meters.
//     // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//     //     // Start at the origin facing the +X direction
//     //     new Pose2d(0, 0, new Rotation2d(0)),
//     //     // Pass through these two interior waypoints, making an 's' curve path
//     //     List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d( 0,-1)), 
//     //     // End 3 meters straight ahead of where we started,% facing forward
//     //     new Pose2d(0, 0, new Rotation2d()),
//     //     config);

//     var thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         trajectory,
//         m_drive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         thetaController,
//         m_drive::setModuleStates,
//         m_drive);

//     // Reset odometry to the starting pose of the trajectory.
//     m_drive.resetOdometry(trajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, true, false));
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }
// }
