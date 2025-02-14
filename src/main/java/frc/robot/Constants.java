// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {

    public static final class DriveConstants {

        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        public static final double kWheelBase = Units.inchesToMeters(26.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARX MAX CAN ID

        public static final int frontLeftDriveId = 22;
        public static final int frontLeftTurningId = 21;
        public static final int frontRightDriveId = 12;
        public static final int frontRightTurningId = 11;

        public static final int backLeftDriveId = 32;
        public static final int backLeftTurningId = 31;
        public static final int backRightDriveId = 2;
        public static final int backRightTurningId = 1;

    }

    public static final class ModuleConstants {

        public static final int kDrivingMotorPinionTeeth = 14;

        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

    }

    public static final class OIConstants {
        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDriveDeadband = 0.2;
    }

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecond = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Units.degreesToRadians(540)
        public static final double kMaxAngularAccelerationRadiansPerSecond = Math.PI; // Units.degreesToRadians(720)

        public static final double kAutoTranslationP = 2.5;
        public static final double kAutoTranslationI = 0;
        public static final double kAutoTranslationD = 0;

        public static final double kAutoRotationP = 1.5;
        public static final double kAutoRotationI = 0;
        public static final double kAutoRotationD = 0;
        
        public static final double kMaxModuleSpeed = 4.8; // Max speed for each module
        public static final double kDriveBaseRadius = 0.46; // Distance from robot center to one module

    }

    public static final class LimelightConstants {
        public static final String kLimelightName = "limelight-balam";
        public static final int[] kValidAprilTagIds = {4};
    }

    public static final class ShuffleboardConstants {
        public static final ShuffleboardTab kSwerveTab = Shuffleboard.getTab("Swerve");
        public static final ShuffleboardTab kVisionTab = Shuffleboard.getTab("Vision");
    }

    public static final class ElevatorConstants {
        public static final int kPrimaryElevatorMotorId = 0;
        public static final int kSecondaryElevatorMotorId = 0;

        public static final int kPrimaryCurrentLimit = 20;
        public static final int kSecondaryCurrentLimit = 20;

        public static final IdleMode kPrimaryIdleMode = IdleMode.kBrake;
        public static final IdleMode kSecondaryIdleMode = IdleMode.kBrake;

        public static final int kMaxHeight = 0;
        public static final int kMinHeight = 0;
    }

    public static final class CoralIntakeConstants {
        public static final int kArmMotorId = 0;
        public static final int kRollerMotorId = 0;

        public static final int kArmCurrentLimit = 20;
        public static final int kRollerCurrentLimit = 20;

        public static final IdleMode kArmIdleMode = IdleMode.kBrake;
        public static final IdleMode kRollerIdleMode = IdleMode.kBrake;

        public static final double kArmMaxPosition = 0;
        public static final double kArmMinPosition = 0;
    }

    public static final class AlgaeIntakeConstants {
        public static final int kPrimaryMotorId = 0;
        public static final int kSecondaryMotorId = 0;

        public static final int kPrimaryCurrentLimit = 20;
        public static final int kSecondaryCurrentLimit = 20;

        public static final IdleMode kPrimaryIdleMode = IdleMode.kBrake;
        public static final IdleMode kSecondaryIdleMode = IdleMode.kBrake;
    }

    public static final class ClimberConstants {
        public static final int kPrimaryMotorId = 0;
        public static final int kSecondaryMotorId = 0;

        public static final int kPrimaryCurrentLimit = 20;
        public static final int kSecondaryCurrentLimit = 20;

        public static final IdleMode kPrimaryIdleMode = IdleMode.kBrake;
        public static final IdleMode kSecondaryIdleMode = IdleMode.kBrake;

        public static final double kMaxEncoderPosition = 0;
        public static final double kMinEncoderPosition = 0;
    }

    public static final class SpecialConstants {
        public static final double PROCESSOR_HEIGHT = 0;
        public static final double SOURCE_HEIGHT = 8.75;
        private static final double L1_HEIGHT = 3;
        private static final double L2_HEIGHT = 5.5;
        private static final double L3_HEIGHT = 21.5;
        
        private static final double PROCESSOR_ANGLE = 0 ;
        private static final double SOURCE_ANGLE = 0.15;
        private static final double L1_ANGLE = 0.3;
        private static final double L2_ANGLE = 0.225;
        private static final double L3_ANGLE = 0.225;
    }

}
