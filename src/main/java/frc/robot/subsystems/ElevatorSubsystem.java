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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  private AbsoluteEncoder m_primaryEncoder;
  @SuppressWarnings("unused")
  private AbsoluteEncoder m_secondaryEncoder;

  private double encoderPosition;

  public static final SparkMaxConfig primaryElevatorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryElevatorConfig = new SparkMaxConfig();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    m_primaryMotor = new SparkMax(ElevatorConstants.kPrimaryElevatorMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(ElevatorConstants.kSecondaryElevatorMotorId, MotorType.kBrushless);

    primaryElevatorConfig
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kPrimaryCurrentLimit);
    secondaryElevatorConfig
      .follow(ElevatorConstants.kPrimaryElevatorMotorId) // Sepa si esto va a funcionar
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kPrimaryCurrentLimit);
    
    m_primaryMotor.configure(primaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_secondaryMotor.configure(secondaryElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_primaryEncoder = m_primaryMotor.getAbsoluteEncoder();
    m_secondaryEncoder = m_secondaryMotor.getAbsoluteEncoder();

  }

  public void setElevatorSpeed(double speed) { // Falta agregar limites para no matar los motores o el elevador
    m_primaryMotor.set(speed);
    m_secondaryMotor.set(speed);
  }

  public void moveElevator(double speed) {
    if (speed > 0 ) {
      if (encoderPosition < ElevatorConstants.kMaxHeight) {
        m_primaryMotor.set(speed);
      } else {
        m_primaryMotor.set(0);
      }
    } else if (speed < 0 ) {
      if (encoderPosition < 0) {
        m_primaryMotor.set(speed);
      } else {
        m_primaryMotor.set(0);
      }
    } else {
      m_primaryMotor.set(0);
    }
  }

  public void stopElevator() {
    m_primaryMotor.set(0);
    m_secondaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_primaryEncoder.getPosition(); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderPosition = m_primaryEncoder.getPosition();
  }
}
