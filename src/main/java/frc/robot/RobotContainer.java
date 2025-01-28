// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




  .______        ___       __          ___      .___  ___. 
  |   _  \      /   \     |  |        /   \     |   \/   | 
  |  |_)  |    /  ^  \    |  |       /  ^  \    |  \  /  | 
  |   _  <    /  /_\  \   |  |      /  /_\  \   |  |\/|  | 
  |  |_)  |  /  _____  \  |  `----./  _____  \  |  |  |  | 
  |______/  /__/     \__\ |_______/__/     \__\ |__|  |__| 
  




*/     

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeShootCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralShootCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  // Drive Controller

  private XboxController m_driverController = new XboxController(OIConstants.kDriveControllerPort);
  private XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private JoystickButton driver_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private JoystickButton driver_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);

  private JoystickButton operator_xButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
  private JoystickButton operator_bButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  private JoystickButton operator_aButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
  private JoystickButton operator_yButton = new JoystickButton(m_operatorController, XboxController.Button.kY.value);

  // Subsystems

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();    
    registedCommands();

    m_robotDrive.zeroHeading();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Elevator Command
    m_elevatorSubsystem.setDefaultCommand(new RunCommand(
      () -> m_elevatorSubsystem.moveElevator(
        -MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband)
      ), m_elevatorSubsystem));

    // Coral Manipulator Arm Command
    m_coralSubsystem.setDefaultCommand(new RunCommand(
      () -> m_coralSubsystem.setArmMotor(
        -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband)
      ), m_coralSubsystem));

    // Climber Command
    m_climberSubsystem.setDefaultCommand(new RunCommand(
      () -> m_climberSubsystem.moveClimber(
        -MathUtil.applyDeadband(m_operatorController.getLeftX(), OIConstants.kDriveDeadband)
      ), m_climberSubsystem));

    // Swerve Drive Command
    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));

  }

  private void configureBindings() {

    // Drive Controller Bindings
    driver_xButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    driver_bButton.onTrue(m_robotDrive.changeDriveModeCmd());

    // Operator Controller Bindings
    operator_aButton.onTrue(new CoralIntakeCommand(m_coralSubsystem)); // A button Intakes Coral
    operator_bButton.onTrue(new CoralShootCommand(m_coralSubsystem)); // B button Shoots Coral
    operator_xButton.onTrue(new AlgaeIntakeCommand(m_algaeSubsystem)); // X button Intakes Algae
    operator_yButton.onTrue(new AlgaeShootCommand(m_algaeSubsystem)); // Y button Shoots Algae

  } 

  private void registedCommands() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
