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
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  private AbsoluteEncoder m_climberEncoder;

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    m_primaryMotor = new SparkMax(ClimberConstants.kPrimaryMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(ClimberConstants.kSecondaryMotorId, MotorType.kBrushless);

    primaryMotorConfig
      .idleMode(ClimberConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ClimberConstants.kPrimaryCurrentLimit);
    secondaryMotorConfig
      .follow(ClimberConstants.kPrimaryMotorId)
      .idleMode(ClimberConstants.kSecondaryIdleMode)
      .smartCurrentLimit(ClimberConstants.kSecondaryCurrentLimit);

      m_primaryMotor.configure(primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void moveClimber(double speed) {
    if (speed > 0) {
      if (getEncoderPosition() < ClimberConstants.kMaxEncoderPosition) {
        m_primaryMotor.set(speed);
      }
    } else if (speed < 0) {
      if (getEncoderPosition() > ClimberConstants.kMinEncoderPosition) {
        m_primaryMotor.set(speed);
      }
    } else {
      m_primaryMotor.set(0);
    }
  }

  public void stopMotors() {
    m_primaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_climberEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
