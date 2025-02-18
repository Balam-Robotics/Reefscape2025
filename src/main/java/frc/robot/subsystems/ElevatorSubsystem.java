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

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  private AbsoluteEncoder m_primaryEncoder;

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    System.out.println("Creates a new ElevatorSubsystem");
    m_primaryMotor = new SparkMax(ElevatorConstants.kPrimaryElevatorMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(ElevatorConstants.kSecondaryElevatorMotorId, MotorType.kBrushless);

    primaryMotorConfig
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kPrimaryCurrentLimit);
    primaryMotorConfig.closedLoop
      .pidf(0.027, 0, 0 , 0.0085);
    secondaryMotorConfig
      .follow(ElevatorConstants.kPrimaryElevatorMotorId, true) // Sepa si esto va a funcionar
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit)
      .inverted(true);
    
    m_primaryMotor.configure(primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_primaryEncoder = m_primaryMotor.getAbsoluteEncoder();
  }

  public void setElevatorPosition(double position) {
    System.out.println("Moving elevator position to " + position);
    m_primaryMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setElevatorSpeed(double speed) {
    System.out.println("Changing elevator speed to " + speed);
    if ((speed < 0 && getEncoderPosition() >= ElevatorConstants.kMinHeight) || (speed > 0 && getEncoderPosition() <= ElevatorConstants.kMaxHeight) ) {
      m_primaryMotor.set(speed);
    }
  }

  public void stopElevator() {
    System.out.println("Stopping Elevator");
    m_primaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_primaryEncoder.getPosition(); 
  }

  public double getEncoderVelocity() {
    return m_primaryEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
