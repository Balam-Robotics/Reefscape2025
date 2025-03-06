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

package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShuffleboardConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class DriveSubsystem extends SubsystemBase {

  // Create each swerve module

  private final BalamSwerveModule m_frontLeft = new BalamSwerveModule(
      DriveConstants.frontLeftDriveId,
      DriveConstants.frontLeftTurningId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final BalamSwerveModule m_frontRight = new BalamSwerveModule(
      DriveConstants.frontRightDriveId,
      DriveConstants.frontRightTurningId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final BalamSwerveModule m_backLeft = new BalamSwerveModule(
      DriveConstants.backLeftDriveId,
      DriveConstants.backLeftTurningId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final BalamSwerveModule m_backRight = new BalamSwerveModule(
      DriveConstants.backRightDriveId,
      DriveConstants.backRightTurningId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Advantage Scope

  private StructArrayPublisher<SwerveModuleState> publish_SwerveStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Measured", SwerveModuleState.struct).publish();

  private StructArrayPublisher<SwerveModuleState> publish_SwerverSetpoints = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Setpoints", SwerveModuleState.struct).publish();

  private StructPublisher<Rotation2d> publish_robotRotation = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/Robot2D", Rotation2d.struct).publish();

  private StructPublisher<Pose2d> publish_robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/RobotPose2D", Pose2d.struct).publish();

  final StructPublisher<Pose2d> publish_poseEstimator = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/PoseEstimation", Pose2d.struct).publish();

  // NavX Gyroscope

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private boolean m_isFieldOriented = true;

  public void changeDriveMode() {
    m_isFieldOriented = !m_isFieldOriented;
  }

  public Command changeDriveModeCmd() {
    return this.runOnce(() -> m_isFieldOriented = !m_isFieldOriented);
  }

  // Odometry

  // private Pose2d limelightPose2d =
  // LimelightHelpers.getBotPose2d_wpiBlue("limelight-balam"); //WIP

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getSwerveModulePositions(),
      new Pose2d(1.21, 5.53, getHeading())); // new Pose2d(1.21, 5.53, getHeading()

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getRotation2d(),
      getSwerveModulePositions(),
      new Pose2d(3.0, 7.0, getRotation2d()));

  private Field2d field;

  // ----------------- Shuffleboard Functions -----------------

  private GenericEntry isConnected = ShuffleboardConstants.kSwerveTab.add("Connected", false).getEntry();
  private GenericEntry isCalibrating = ShuffleboardConstants.kSwerveTab.add("Calibrating", false).getEntry();
  private GenericEntry gyroYaw = ShuffleboardConstants.kSwerveTab.add("Yaw", 0).getEntry();
  private GenericEntry gyroPitch = ShuffleboardConstants.kSwerveTab.add("Pitch", 0).getEntry();
  private GenericEntry gyroRoll = ShuffleboardConstants.kSwerveTab.add("Roll", 0).getEntry();
  private GenericEntry gyroAngle = ShuffleboardConstants.kSwerveTab.add("Angle", 0).getEntry();
  private GenericEntry isMoving = ShuffleboardConstants.kSwerveTab.add("Moving", false).getEntry();
  private GenericEntry isRotating = ShuffleboardConstants.kSwerveTab.add("Rotating", false).getEntry();
  private GenericEntry isFieldOrientedEntry = ShuffleboardConstants.kSwerveTab.add("Is Field Oriented", false)
      .getEntry();
  private GenericEntry limeligtTXValue = ShuffleboardConstants.kSwerveTab.add("Limelight TX", 0).getEntry();
  private GenericEntry limeligtTVValue = ShuffleboardConstants.kSwerveTab.add("Limelight TV", false).getEntry();

  private void updateShuffleboard() {
    isConnected.setBoolean(m_gyro.isConnected());
    isCalibrating.setBoolean(m_gyro.isCalibrating());
    gyroYaw.setDouble(Math.round(m_gyro.getYaw()));
    gyroPitch.setDouble(Math.round(m_gyro.getPitch()));
    gyroRoll.setDouble(Math.round(m_gyro.getRoll()));
    gyroAngle.setDouble(Math.round(m_gyro.getAngle()));
    isMoving.setBoolean(m_gyro.isMoving());
    isRotating.setBoolean(m_gyro.isRotating());
    isFieldOrientedEntry.setBoolean(m_isFieldOriented);

    limeligtTXValue.setDouble(LimelightHelpers.getTX(LimelightConstants.kLimelightName));
    limeligtTVValue.setBoolean(LimelightHelpers.getTV(LimelightConstants.kLimelightName));
  }

  // ----------------- Drive Subsystem Functions -----------------

  // Gyro Functions

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  // Odometry Functions

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void resetPose(Pose2d reseted) {
    m_odometry.resetPosition(
        getRotation2d(),
        getSwerveModulePositions(),
        reseted);

    poseEstimator.resetPosition(
        getRotation2d(),
        getSwerveModulePositions(),
        reseted);
  }

  public void zeroPose() {
    // m_odometry.resetPosition(getHeading(), getSwerveModulePositions(), new
    // Pose2d(1.21, 5.53, getHeading()));
    m_odometry.resetPosition(getHeading(), getSwerveModulePositions(), getPose());
  }

  // Relative Robot DriveSubsystem for Pathplanner

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void setChassisSpeed(ChassisSpeeds desired) {
    SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desired);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, 4.8);
    setDesiredStates(newStates);
  }

  // Relative and FieldOriented DriveSubsystem with Controller Inputs

  public void drive(double xSpeed, double ySpeed, double rot, boolean overrideFieldOriented) {

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    boolean fieldOriented;

    if (overrideFieldOriented) {
      fieldOriented = true;
    } else {
      fieldOriented = m_isFieldOriented;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setDesiredStates(swerveModuleStates);

  }

  // Misc

  public void setX() {
    m_frontLeft.setdesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_backLeft.setdesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setdesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setdesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Swerve Common Functions

  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.setdesiredState(swerveModuleStates[0]);
    m_frontRight.setdesiredState(swerveModuleStates[1]);
    m_backLeft.setdesiredState(swerveModuleStates[2]);
    m_backRight.setdesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    };
  }

  public SwerveModuleState[] getSwerveModuleSetpoints() {
    return new SwerveModuleState[] {
        m_frontLeft.getSetpoints(),
        m_frontRight.getSetpoints(),
        m_backLeft.getSetpoints(),
        m_backRight.getSetpoints()
    };
  }

  public ChassisSpeeds autoAlign(String direction) {
    double tx = LimelightHelpers.getTX(LimelightConstants.kLimelightName);
    boolean tv = LimelightHelpers.getTV(LimelightConstants.kLimelightName);
    double kP_rotation = 0.02;
    double strafeSpeed = 0.5;

    if (!tv) {
      return new ChassisSpeeds(0, 0, 0);
    }

    // Align with target tag
    double rotationSpeed = -kP_rotation * tx;
    rotationSpeed = MathUtil.clamp(rotationSpeed, -1.0, 1.0);

    // Strafe to target tag
    double strafe = 0.0;
    if (Math.abs(tx) < 2.0) {

      if (direction == "left") {
        strafe = -strafeSpeed;
      } else if (direction == "right") {
        strafe = strafeSpeed;
      }

    }

    return new ChassisSpeeds(0.0, strafe, rotationSpeed);

  }

  // Drive Subsystem Constructor and Periodic

  public DriveSubsystem() {

    // Elastic Test

    SmartDashboard.putData("Swerve",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> m_backRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> getHeading().getRadians(), null);
        });

    SmartDashboard.putData("Swerve Setpoints",
      builder -> {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontRight.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getSetpoints().speedMetersPerSecond, null);
      });

    // Shuffleboard 2D Field

    field = new Field2d();
    ShuffleboardConstants.kSwerveTab.add("Field", field);

    // Pathplanner trajectory for AdvantageScope

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // Pathplanner

    // RobotConfig config = new RobotConfig(0, 0, null, null);
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = new RobotConfig(0, 0, null, null);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRelativeChassisSpeeds,
        (speeds, feedforwards) -> setChassisSpeed(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(
                AutoConstants.kAutoTranslationP,
                AutoConstants.kAutoTranslationI,
                AutoConstants.kAutoTranslationD),
            new PIDConstants(
                AutoConstants.kAutoRotationP,
                AutoConstants.kAutoRotationI,
                AutoConstants.kAutoRotationD)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

  }

  @Override
  public void periodic() {

    // Odometry Update

    m_odometry.update(getHeading(), getSwerveModulePositions());

    poseEstimator.update(getRotation2d(), getSwerveModulePositions());

    field.setRobotPose(m_odometry.getPoseMeters());

    // Publish Advantage Scope Data and Shuffleboard Data

    SwerveModuleState[] physicPoints = getSwerveModuleStates();

    SwerveModuleState[] setPoints = getSwerveModuleSetpoints();

    publish_SwerveStates.set(physicPoints);
    publish_SwerverSetpoints.set(setPoints);
    publish_robotRotation.set(getRotation2d());
    publish_robotPose.set(getPose());
    publish_poseEstimator.set(poseEstimator.getEstimatedPosition());
    updateShuffleboard();

  }

}
