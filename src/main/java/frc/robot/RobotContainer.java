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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpecialConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Maldito;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  // Drive Controller

  //private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriveControllerPort);
  private CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  //private Trigger driver_xButton = m_driverController.x();
  //private Trigger driver_bButton = m_driverController.b();

  private Trigger operator_xButton = m_operatorController.x();
  private Trigger operator_yButton = m_operatorController.y();
  private Trigger operator_bButton = m_operatorController.b();
  private Trigger operator_aButton = m_operatorController.a();

  private Trigger operator_leftBumper = m_operatorController.leftBumper();
  private Trigger operator_leftTrigger = m_operatorController.leftTrigger();
  private Trigger operator_leftStick = m_operatorController.leftStick();
  private Trigger operator_startButton = m_operatorController.start();
  private Trigger operator_POVDown = m_operatorController.povDown();

  // Subsystems

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public TestSubsystem testSubsystem = new TestSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();    
    registedCommands();

    m_robotDrive.zeroHeading();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Swerve Drive Command
    /*
    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));
 */
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

  //Command manualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorSpeed(-m_operatorController.getLeftY() * 0.5), m_elevatorSubsystem);

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

  // Test Command

  Command runCommandTest = new RunCommand(() -> testSubsystem.runCommandTest(), testSubsystem);
  Command startEndCommandTest = new StartEndCommand(() -> testSubsystem.startCommandTest(), () -> testSubsystem.endCommandTest(), testSubsystem);
  //Command run1CommandTest = new RunCommand(() -> testSubsystem.paralelCommadnTest(), testSubsystem);
  //Command run2CommandTest = new RunCommand(() -> testSubsystem.paralel2CommadnTest(), testSubsystem);
  //ParallelCommandGroup paralelCommadnGroupTest = new ParallelCommandGroup(run1CommandTest, run2CommandTest);  

  public Command mamada() {
    return Commands.runOnce(() -> new PrintCommand("null"));
  }

  private void configureBindings() {

    // Drive Controller Bindings
    //driver_xButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    //driver_bButton.onTrue(m_robotDrive.changeDriveModeCmd());

    // Operator Controller Bindings
    /*
    operator_aButton.onTrue(new PrintCommand("A Button"));
    operator_xButton.onTrue(new PrintCommand("X Button"));
    operator_yButton.onTrue(new PrintCommand("Y Button"));
    operator_bButton.onTrue(new PrintCommand("B Button"));
    operator_leftBumper.onTrue(new PrintCommand("Left Bumper"));
    operator_startButton.onTrue(new PrintCommand("Start Button"));
    operator_leftTrigger.whileTrue(new PrintCommand("Left Trigger"));
    operator_leftStick.whileTrue(new PrintCommand("Left Stick"));*/
    operator_POVDown.whileTrue(new PrintCommand("POV Down"));
     

     //operator_aButton.onTrue(new Maldito(testSubsystem));
     operator_xButton.onTrue(new PrintCommand("X Button"));
     operator_xButton.onTrue(new Maldito(testSubsystem));
     m_operatorController.leftBumper().onTrue(runCommandTest);
     //operator_aButton.whileTrue(runCommandTest);
     Command idiota = new PrintCommand("null");
     Command idiota2 = new RunCommand(() -> System.out.println("FUNCIONA PTM"), testSubsystem);
     Command idiota3 = new RunCommand(() -> new PrintCommand("null"), testSubsystem);
     Command mad = mamada();
    m_operatorController.b().onTrue(Commands.runOnce(() -> new PrintCommand("null").ignoringDisable(true)));
     //operator_aButton.onTrue(mad);
     //operator_startButton.onTrue(paralelCommadnGroupTest);

  } 

  private void registedCommands() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().withTimeout(0.2);
  }
}
