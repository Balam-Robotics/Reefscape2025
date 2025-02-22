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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpecialConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  // Drive Controller

  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriveControllerPort);
  private CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

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

    // Swerve Drive Command
    
    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));
 
  }

  // Start up and set up the commands

  // Processor Commands
  Command liftToProcessorCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.PROCESSOR_HEIGHT), m_elevatorSubsystem);
  Command wristToProcessorCommand = new RunCommand(() -> m_coralSubsystem.setWristAngle(SpecialConstants.PROCESSOR_ANGLE), m_coralSubsystem);
  ParallelCommandGroup processorCommandGroup = new ParallelCommandGroup(liftToProcessorCommand, wristToProcessorCommand);

  // Source Commands
  Command liftToSourceCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.SOURCE_HEIGHT), m_elevatorSubsystem);
  Command wristToSourceCommand = new RunCommand(() -> m_coralSubsystem.setWristAngle(SpecialConstants.SOURCE_ANGLE), m_coralSubsystem);
  ParallelCommandGroup sourceCommandGroup = new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);

  // L1 Commands

  Command liftToL1Command = new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L1_ANGLE), m_elevatorSubsystem);
  Command wristToL1Command = new RunCommand(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L1_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l1CommandGroup = new ParallelCommandGroup(liftToL1Command, wristToL1Command);  
  
  // L2 Commands

  Command liftToL2Command = new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L2_ANGLE), m_elevatorSubsystem);
  Command wristToL2Command = new RunCommand(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L2_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l2CommandGroup = new ParallelCommandGroup(liftToL2Command, wristToL2Command);  
  
  // L3 Commands

  Command liftToL3Command = new RunCommand(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L3_ANGLE), m_elevatorSubsystem);
  Command wristToL3Command = new RunCommand(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L3_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l3CommandGroup = new ParallelCommandGroup(liftToL3Command, wristToL3Command);  

  // Manual Lift
  Command test = Commands.runOnce(() -> System.out.println("Test"));
  Command manualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorSpeed(-m_operatorController.getLeftY() * 0.5), m_elevatorSubsystem);

  // Climber Commands

  Command climbUpCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbDownCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(-0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbHoldCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.1), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);

  // Algae Commands

  Command intakeAlgaeCommand = new StartEndCommand(() -> m_algaeSubsystem.intakeAlgae(), () -> m_algaeSubsystem.stopAlgae(), m_algaeSubsystem);
  Command ejectAlgaeCommand = new StartEndCommand(() -> m_algaeSubsystem.ejectAlgae(), () -> m_algaeSubsystem.stopAlgae(), m_algaeSubsystem);

  // Intake Commands

  Command intakeCoralCommand = new StartEndCommand(() -> m_coralSubsystem.intakeCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem);
  Command ejectCoralCommand = new StartEndCommand(() -> m_coralSubsystem.ejectCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem);  

  private void configureBindings() {


    // Drive Controller Bindings
    m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    m_driverController.b().onTrue(m_robotDrive.changeDriveModeCmd());

    m_driverController.povUp().whileTrue(climbUpCommand);
    m_driverController.povDown().whileTrue(climbDownCommand);
    m_driverController.povLeft().whileTrue(climbHoldCommand);

  
    // Operator Controller Bindings

    m_operatorController.rightBumper().whileTrue(ejectAlgaeCommand);
    m_operatorController.rightTrigger().whileTrue(intakeAlgaeCommand);

    m_operatorController.leftBumper().whileTrue(ejectCoralCommand);
    m_operatorController.leftTrigger().whileTrue(intakeCoralCommand);

    m_operatorController.povDown().onTrue(processorCommandGroup).debounce(1);
    m_operatorController.povLeft().onTrue(sourceCommandGroup);

    m_operatorController.a().onTrue(l1CommandGroup);
    m_operatorController.b().onTrue(l2CommandGroup);
    m_operatorController.y().onTrue(l3CommandGroup);
    m_operatorController.x().onTrue(test);

    m_operatorController.start().whileTrue(manualLiftCommand);

  } 

  private void registedCommands() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().withTimeout(0.2);
  }
}
