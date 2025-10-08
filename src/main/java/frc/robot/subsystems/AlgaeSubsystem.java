// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




        :::::::::      :::     :::            :::       :::   :::          ::::::::  ::::::::::  ::::::::  ::::::::::: 
     :+:    :+:   :+: :+:   :+:          :+: :+:    :+:+: :+:+:        :+:    :+: :+:    :+: :+:    :+: :+:     :+:  
    +:+    +:+  +:+   +:+  +:+         +:+   +:+  +:+ +:+:+ +:+              +:+ +:+              +:+         +:+    
   +#++:++#+  +#++:++#++: +#+        +#++:++#++: +#+  +:+  +#+           +#++:  +#++:++#+      +#+          +#+      
  +#+    +#+ +#+     +#+ +#+        +#+     +#+ +#+       +#+              +#+        +#+   +#+           +#+        
 #+#    #+# #+#     #+# #+#        #+#     #+# #+#       #+#       #+#    #+# #+#    #+#  #+#           #+#          
#########  ###     ### ########## ###     ### ###       ###        ########   ########  ##########     ###   
  




*/    

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class AlgaeSubsystem extends SubsystemBase {

  private SparkMax m_PrimaryIntakeMotor;
  private SparkMax m_SecondaryIntakeMotor;
  private SparkMax m_AlgaeWristMotor;

  private AbsoluteEncoder m_encoder;

  private static final SparkMaxConfig primaryIntakeMotorConfig = new SparkMaxConfig();
  private static final SparkMaxConfig secondaryIntakeMotorConfig = new SparkMaxConfig();
  private static final SparkMaxConfig algaeWristMotorConfig = new SparkMaxConfig();

  public boolean isEjectingAlgae = false;
  public boolean isIntakingAlgae = false;
  public boolean isWristMoving = false;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {

    m_PrimaryIntakeMotor = new SparkMax(AlgaeConstants.kPrimaryMotorId, MotorType.kBrushless);
    m_SecondaryIntakeMotor = new SparkMax(AlgaeConstants.kSecondaryMotorId, MotorType.kBrushless);
    m_AlgaeWristMotor = new SparkMax(AlgaeConstants.kWristMotorId, MotorType.kBrushless);

    primaryIntakeMotorConfig
    .idleMode(AlgaeConstants.kPrimaryIdleMode)
    .smartCurrentLimit(AlgaeConstants.kPrimaryCurrentLimit);
    secondaryIntakeMotorConfig
    .follow(AlgaeConstants.kPrimaryMotorId, true)
    .idleMode(AlgaeConstants.kSecondaryIdleMode)
    .smartCurrentLimit(AlgaeConstants.kSecondaryCurrentLimit);
    algaeWristMotorConfig
    .idleMode(AlgaeConstants.kWristIdleMode)
    .smartCurrentLimit(AlgaeConstants.kWristCurrentLimit);;
    algaeWristMotorConfig.closedLoop
    .pidf(AlgaeConstants.kWristPIDkP, AlgaeConstants.kWristPIDkI, AlgaeConstants.kWristPIDkD, 0);
    
    m_PrimaryIntakeMotor.configure(primaryIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_SecondaryIntakeMotor.configure(secondaryIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_AlgaeWristMotor.configure(algaeWristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_AlgaeWristMotor.getAbsoluteEncoder();

  }

  public void setWristAngle(double position) {
    m_AlgaeWristMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setDEBUGWristAngle() {
    m_AlgaeWristMotor.getClosedLoopController().setReference(algaePIDEntry.getDouble(0.0), ControlType.kPosition);
  }

  public void setWristSpeed(double speed) {
    // TODO: TEST LIMITS SUPPORT
    if (speed > 0 && getEncoderPosition() >= AlgaeConstants.kWristMaxPosition) {
      m_AlgaeWristMotor.set(0); // Stop the motor if exceeding max position
    } else if (speed < 0 && getEncoderPosition() <= AlgaeConstants.kWristMinPosition) {
      m_AlgaeWristMotor.set(0); // Stop the motor if below min position
    } else {
      m_AlgaeWristMotor.set(speed); // Set the motor speed if within limits
    }
  }

  private GenericEntry algaePIDEntry = ShuffleboardConstants.kAlgaeTab.add("Algae PID", 0.0)
  .withSize(2, 1)
  .withPosition(2, 1)
  .withProperties(Map.of("show_submit_button", true))
  .getEntry();
  private GenericEntry algaePosition = ShuffleboardConstants.kAlgaeTab.add("Algae Wrist Angle", 0.0)
  .withWidget(BuiltInWidgets.kGraph)
  .withSize(2, 2) 
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry algaeVelocity = ShuffleboardConstants.kAlgaeTab.add("Algae Wrist Velocity", 0.0)
  .withWidget(BuiltInWidgets.kAccelerometer)
  .withSize(2, 1)
  .withPosition(2, 0) 
  .getEntry();
  

  private ShuffleboardLayout valuesLayout = ShuffleboardConstants.kAlgaeTab
  .getLayout("Algae Booleans", BuiltInLayouts.kList)
  .withSize(2, 3)
  .withPosition(6, 0);

  {
    valuesLayout.addBoolean("Ejecting Algae", () -> isEjectingAlgae);
    valuesLayout.addBoolean("Intaking Algae", () -> isIntakingAlgae);
  }
 

  private GenericEntry wristVoltage = ShuffleboardConstants.kAlgaeTab.add("Wrist Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry algaeVoltage = ShuffleboardConstants.kAlgaeTab.add("Intake Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 1)
  .getEntry();

  public void intakeAlgae() {
    isIntakingAlgae = true;
    isEjectingAlgae = false;
    m_PrimaryIntakeMotor.set(1);
  }

  public void ejectAlgae() {
    isEjectingAlgae = true;
    isIntakingAlgae = false;
    m_PrimaryIntakeMotor.set(-1);
  }

  public void stopIntake() {
    isEjectingAlgae = false;
    isIntakingAlgae = false;
    m_PrimaryIntakeMotor.set(0);
  }
  
  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }

  public double getEncoderVelocity() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    algaePosition.setDouble(getEncoderPosition());
    algaeVelocity.setDouble(getEncoderVelocity());
    wristVoltage.setDouble(m_AlgaeWristMotor.getOutputCurrent());
    algaeVoltage.setDouble(m_PrimaryIntakeMotor.getOutputCurrent());
    //System.out.println(Units.radiansToDegrees(getEncoderPosition()));
  }
}
