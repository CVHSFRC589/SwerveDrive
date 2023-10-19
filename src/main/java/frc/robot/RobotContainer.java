// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AUTO.NOTHING;
import frc.robot.commands.AUTO.ScoreBalance;
import frc.robot.commands.AUTO.ScoreGetBalance;
import frc.robot.commands.DRIVE.AutoAlign;
import frc.robot.commands.DRIVE.DefaultDrive;
import frc.robot.commands.DRIVE.ResetGyro;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SpinnersSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        // private final static SpinnersSubsystem m_robotSpin = new SpinnersSubsystem();
        // private final Gyro m_Gyro = new Gyro();
        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        XboxController m_codriverController = new XboxController(OIConstants.kCODriverControllerPort);
        private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Command test = m_autoBuilder.fullAuto(new PathPlannerTrajectory());

                m_autoChooser.addOption("Drive out and Balance", new
                ScoreGetBalance(m_robotDrive));
                // m_autoChooser.addOption("Score Backup Balance", new ScoreBalance(m_robotDrive, m_robotSpin));
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

                // ===============DRIVER========================//
                // new JoystickButton(m_driverController, 3)
                // .toggleOnTrue(new AutoBalanceX(m_robotDrive));
                new JoystickButton(m_driverController, 4)
                                .toggleOnTrue(new ResetGyro(m_robotDrive));
                new JoystickButton(m_driverController, 1)
                                .toggleOnTrue(new AutoAlign(m_robotDrive));

                new JoystickButton(m_driverController, 2)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));
                // ===============CO-DRIVER========================//
                //

        }

        public Command getAutonomousCommand() {

                return m_autoChooser.getSelected();

        }

}
