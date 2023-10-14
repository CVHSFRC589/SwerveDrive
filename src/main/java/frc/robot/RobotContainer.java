// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AUTO.NOTHING;
import frc.robot.commands.AUTO.ScoreGetBalance;
import frc.robot.commands.DRIVE.AutoAlign;
import frc.robot.commands.DRIVE.AutoBalanceX;
import frc.robot.commands.DRIVE.DefaultDrive;
import frc.robot.commands.DRIVE.ResetGyro;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();

        // private final Gyro m_Gyro = new Gyro();
        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        private static HashMap<String, Command> m_eventMap = new HashMap<>();
        private HashMap<String, Auton> autoMap = new HashMap<String, Auton>();
        private PathPlanner m_pathCreator;
        public final static SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
                        () -> m_robotDrive.getPose(),
                        pose -> m_robotDrive.resetOdometry(pose),
                        new PIDConstants(Constants.ModuleConstants.kDrivingP,
                                        Constants.ModuleConstants.kDrivingI,
                                        Constants.ModuleConstants.kDrivingD),
                        new PIDConstants(Constants.ModuleConstants.kTurningP,
                                        Constants.ModuleConstants.kTurningI,
                                        Constants.ModuleConstants.kTurningD),
                        chassisSpeeds -> {
                                SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics
                                                .toSwerveModuleStates(chassisSpeeds);
                                m_robotDrive.setFrontLeftModuleState(moduleStates[0]);
                                m_robotDrive.setFrontRightModuleState(moduleStates[1]);
                                m_robotDrive.setRearLeftModuleState(moduleStates[2]);
                                m_robotDrive.setRearRightModuleState(moduleStates[3]);
                        },
                        m_eventMap,
                        m_robotDrive);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_eventMap.put("Grab", new NOTHING());
                m_eventMap.put("Throw", new NOTHING());
                m_eventMap.put("ArmDown", new NOTHING());
                // Command test = m_autoBuilder.fullAuto(new PathPlannerTrajectory());

                m_autoChooser.addOption("Drive out and Balance", new ScoreGetBalance(m_robotDrive));
                m_autoChooser.setDefaultOption("NOTHING", new NOTHING());
                SmartDashboard.putData(m_autoChooser);

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2),
                                                OIConstants.kDriveDeadband)));
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                // new RunCommand(
                // () -> m_robotDrive.drive%(
                // -MathUtil.applyDeadband(m_driverController.getLeftY(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(m_driverController.getLeftX(),
                // OIConstants.kDriveDeadband),
                // -MathUtil.applyDeadband(
                // m_driverController.getRawAxis(2),
                // OIConstants.kDriveDeadband),
                // true, true),
                // m_robotDrive));
        }

        private void configureButtonBindings() {
                new JoystickButton(m_driverController, 3)
                                .toggleOnTrue(new AutoBalanceX(m_robotDrive));
                new JoystickButton(m_driverController, 4)
                                .toggleOnTrue(new ResetGyro(m_robotDrive));
                new JoystickButton(m_driverController, 1)
                                .toggleOnTrue(new AutoAlign(m_robotDrive));

                new JoystickButton(m_driverController, 2)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));
        }

        public Command getAutonomousCommand() {

                return m_autoChooser.getSelected();

        }

}
