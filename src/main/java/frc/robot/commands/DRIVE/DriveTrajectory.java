package frc.robot.commands.DRIVE;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.DriveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class DriveTrajectory extends CommandBase {
//   private final Trajectory m_trajectory;
//   private final DriveSubsystem m_drive;
//   private final ProfiledPIDController m_thetaController;
// A
//   public DriveTrajectory(Trajectory trajectory, DriveSubsystem drive, double maxspeed, double maxaccel) {
//     // Create config for trajectory
//     m_drive = drive;
//     m_trajectory = trajectory;
//     m_thetaController = new ProfiledPIDController(
//         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     TrajectoryConfig config = new TrajectoryConfig(maxspeed, maxaccel);

//     // An example trajectory to follow. All units in meters.
//     // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//     // // Start at the origin facing the +X direction
//     // new Pose2d(0, 0, new Rotation2d(0)),
//     // // Pass through these two interior waypoints, making an 's' curve path
//     // List.of(new Translation2d(1, 1), new Translation2d(2, 0), new Translation2d(
//     // 0,-1)),
//     // // End 3 meters straight ahead of where we started,% facing forward
//     // new Pose2d(0, 0, new Rotation2d()),
//     // config);

//     // Reset odometry to the starting pose of the trajectory.
//     m_drive.resetOdometry(trajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         m_trajectory,
//         m_drive::getPose, // Functional interface to feed supplier
//         DriveConstants.kDriveKinematics,

//         // Position controllers
//         new PIDController(AutoConstants.kPXController, 0, 0),
//         new PIDController(AutoConstants.kPYController, 0, 0),
//         m_thetaController,
//         m_drive::setModuleStates,
//         m_drive);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
