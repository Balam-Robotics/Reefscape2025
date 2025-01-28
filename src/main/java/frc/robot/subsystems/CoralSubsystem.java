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
import frc.robot.Constants.CoralIntakeConstants;

public class CoralSubsystem extends SubsystemBase {

  private SparkMax m_armMotor;
  private SparkMax m_rollerMotor;

  private AbsoluteEncoder m_armEncoder;

  public static final SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {

    m_armMotor = new SparkMax(CoralIntakeConstants.kArmMotorId, MotorType.kBrushless);
    m_rollerMotor = new SparkMax(CoralIntakeConstants.kRollerMotorId, MotorType.kBrushless);

    armMotorConfig
      .idleMode(CoralIntakeConstants.kArmIdleMode)
      .smartCurrentLimit(CoralIntakeConstants.kArmCurrentLimit);
    rollerMotorConfig
      .idleMode(CoralIntakeConstants.kRollerIdleMode)
      .smartCurrentLimit(CoralIntakeConstants.kRollerCurrentLimit);

      m_armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setArmMotor(double speed) {
    if (speed > 0 ) {
      if (getEncoderPosition() < CoralIntakeConstants.kArmMaxPosition) {
        m_armMotor.set(speed);
      } else {
        m_armMotor.set(0);
      }
    } else if (speed < CoralIntakeConstants.kArmMinPosition) {
      if (getEncoderPosition() < 0) {
        m_armMotor.set(speed);
      } else {
        m_armMotor.set(0);
      }
    } else {
      m_armMotor.set(0);
    }
  }
  public void stopArmMotor() {
    m_armMotor.set(0);
  }

  public void intakeCoral() {
    m_rollerMotor.set(1);
  }

  public void shootCoral() {
    m_rollerMotor.set(-1);
  }

  public void stopRollerMotor() {
    m_rollerMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
