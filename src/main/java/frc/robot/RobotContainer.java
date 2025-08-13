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


import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.Constants.SpecialConstants;
import frc.robot.commands.AutoEjectCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.util.CameraSystem;
import frc.robot.util.GameTimer;

public class RobotContainer {

  // Drive Controller

  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDebug ? OIConstants.kOperatorControllerPort : OIConstants.kDriveControllerPort);
  private CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kDebug ? OIConstants.kDriveControllerPort : OIConstants.kOperatorControllerPort);

  // Subsystems

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  @SuppressWarnings("unused")
  private CameraSystem m_cameraSystem = new CameraSystem(ShuffleboardConstants.kSwerveTab);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();    
    registedCommands();
    setupDriverTab();

    m_robotDrive.zeroHeading();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Swerve Drive Command

    m_robotDrive.setDefaultCommand(Commands.runOnce(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));
 
  }

  // Start up and set up the commands //

  // Source Commands
  Command liftToSourceCommand = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.SOURCE_HEIGHT), m_elevatorSubsystem);
  Command wristToSourceCommand = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.SOURCE_ANGLE), m_coralSubsystem);
  ParallelCommandGroup sourceCommandGroup = new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);

  // L1 Commands

  Command liftToL1Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L1_HEIGHT), m_elevatorSubsystem);
  Command wristToL1Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L1_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l1CommandGroup = new ParallelCommandGroup(liftToL1Command, wristToL1Command);  
  
  // L2 Commands

  Command liftToL2Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L2_HEIGHT), m_elevatorSubsystem);
  Command wristToL2Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L2_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l2CommandGroup = new ParallelCommandGroup(liftToL2Command, wristToL2Command);  
  
  // L3 Commands

  Command liftToL3Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L3_HEIGHT), m_elevatorSubsystem);
  Command wristToL3Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L3_ANGLE), m_coralSubsystem);
  ParallelCommandGroup l3CommandGroup = new ParallelCommandGroup(liftToL3Command, wristToL3Command);  

  // Reset Elevator Position 

  Command resetElevatorCommand = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0), m_elevatorSubsystem);
  Command resetWristCommand = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.DEFAULT_ANGLE), m_coralSubsystem);
  ParallelCommandGroup resetCommandGroup = new ParallelCommandGroup(resetElevatorCommand, resetWristCommand);

  // Manual Lift
  Command manualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorSpeed(MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband) * 0.5), m_elevatorSubsystem);
  Command stopManualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.stopElevator(), m_elevatorSubsystem);

  // PID Controllers
  Command pidLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setShuffleboardPIDElevator(), m_elevatorSubsystem);
  Command pidWristCommand = new RunCommand(() -> m_coralSubsystem.setShuffleboardPIDWrist(), m_coralSubsystem);

  // Climber Commands

  Command climbUpCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbDownCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(-0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbHoldCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.1), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);

  // Intake Commands

  Command intakeCoralCommand = new StartEndCommand(() -> m_coralSubsystem.intakeCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem);
  Command ejectCoralCommand = new StartEndCommand(() -> m_coralSubsystem.ejectCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem);  

  // Auto-Align Commands

  //Command leftAutoAlignCommand = new StartEndCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("left")), () -> m_robotDrive.setChassisSpeed(new ChassisSpeeds(0, 0, 0)), m_robotDrive);
  //Command rightAutoAlignCommand = new StartEndCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("right")), () -> m_robotDrive.setChassisSpeed(new ChassisSpeeds(0, 0, 0)), m_robotDrive);

  Command leftAutoAlightCommand = new RunCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("left")), m_robotDrive);
  Command rightAutoAlightCommand = new RunCommand(() ->m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("right")), m_robotDrive);

  //Pathplaner commands

  Command resetGyroCommand = Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive);

  private void configureBindings() {

    // Drive Controller Bindings
    m_driverController.x().whileTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.b().onTrue(m_robotDrive.changeDriveModeCmd());
 
    
    if (OIConstants.kDemo) { return; }

    m_driverController.povLeft().whileTrue(leftAutoAlightCommand);
    m_driverController.povRight().whileTrue(rightAutoAlightCommand);

    m_driverController.povUp().whileTrue(climbUpCommand);
    m_driverController.povDown().whileTrue(climbDownCommand);
    //m_driverController.povLeft().whileTrue(climbHoldCommand);

    // Operator Controller Bindings

    m_operatorController.leftBumper().whileTrue(ejectCoralCommand);
    m_operatorController.leftTrigger().whileTrue(intakeCoralCommand);
   
    m_operatorController.y().onTrue(l3CommandGroup); // Level 3
    m_operatorController.b().onTrue(l2CommandGroup); // Level 2
    m_operatorController.a().onTrue(l1CommandGroup); // Level 1
    m_operatorController.x().onTrue(resetCommandGroup); // Reset Elevator Position
    m_operatorController.povLeft().onTrue(sourceCommandGroup); // Source Command

    m_operatorController.start().whileTrue(manualLiftCommand);
    m_operatorController.start().whileFalse(stopManualLiftCommand);    
  
    if (OIConstants.kDebug) {
      ParallelCommandGroup debugCommandGroup = new ParallelCommandGroup(pidLiftCommand, pidWristCommand);
      m_operatorController.povUp().whileTrue(pidLiftCommand);
      //m_operatorController.povUp().whileTrue(pidLiftCommand);
      //m_operatorController.povDown().whileTrue(pidWristCommand);
    }

  } 

  private void registedCommands() {

    NamedCommands.registerCommand("l1Command", l1CommandGroup);
    NamedCommands.registerCommand("l2Command", l2CommandGroup);
    NamedCommands.registerCommand("l3Command", l3CommandGroup);
    NamedCommands.registerCommand("sourceCommand", sourceCommandGroup);
    NamedCommands.registerCommand("intakeCoral", new AutoIntakeCommand(m_coralSubsystem).withTimeout(3));
    NamedCommands.registerCommand("ejectCoral", new AutoEjectCommand(m_coralSubsystem).withTimeout(1 ));
    NamedCommands.registerCommand("resetGyro", resetGyroCommand);
  
  }

  private void setupDriverTab() {
    
    GameTimer gameTimer = new GameTimer();
    SendableRegistry.add(gameTimer, "Match Time");
    ShuffleboardConstants.kSwerveTab.add(gameTimer)
    .withWidget("Match Time")
    .withProperties(Map.of("Font color", "black"))
    .withSize(2, 1);

  }

  public Command getAutonomousCommand() {
    if (OIConstants.kDemo) { 
      SequentialCommandGroup prueba = new SequentialCommandGroup(liftToSourceCommand, new WaitCommand(1.5), liftToL1Command, new WaitCommand(1.5));
      SequentialCommandGroup yes = new SequentialCommandGroup(prueba, prueba, prueba);

      return yes;
    }

    return autoChooser.getSelected();
  }
}
