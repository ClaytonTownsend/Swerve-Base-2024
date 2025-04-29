// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

  // Create a driver controller
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  // Create a manipulator controller
  private final XboxController m_manipulatorController = new XboxController(OperatorConstants.kManipulatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_SwerveSubsystem.setDefaultCommand(
      
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
        () -> m_SwerveSubsystem.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
          true),
          m_SwerveSubsystem));
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_SwerveSubsystem.setX(),
            m_SwerveSubsystem));



    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        // TODO: Replace with actual autonomous command or command group
      return new RunCommand(
        () -> m_SwerveSubsystem.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
          true),
          m_SwerveSubsystem);
  }
}
