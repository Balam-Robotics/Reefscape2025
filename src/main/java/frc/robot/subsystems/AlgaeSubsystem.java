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

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {

    m_primaryMotor = new SparkMax(AlgaeIntakeConstants.kPrimaryMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(AlgaeIntakeConstants.kSecondaryMotorId, MotorType.kBrushless);

    primaryMotorConfig
      .idleMode(AlgaeIntakeConstants.kPrimaryIdleMode)
      .smartCurrentLimit(AlgaeIntakeConstants.kPrimaryCurrentLimit);
    secondaryMotorConfig.
      idleMode(AlgaeIntakeConstants.kSecondaryIdleMode)
      .smartCurrentLimit(AlgaeIntakeConstants.kSecondaryCurrentLimit);

    m_primaryMotor.configure(primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void intakeAlgae() {
    m_primaryMotor.set(1);
    m_secondaryMotor.set(-1);
  }

  public void shootAlgae() {
    m_primaryMotor.set(-1);
    m_secondaryMotor.set(1);
  }

  public void stopMotors() {
    m_primaryMotor.set(0);
    m_secondaryMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
