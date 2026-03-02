// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // ✅ Drive inversion (selon ton test réel) :
  // AG = false, AD = true, RG = false, RD = true
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      false); // AG

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      true); // AD (inversé)

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      false); // RG

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      true); // RD (inversé)

  // Gyro (Pigeon2 - Phoenix 6)
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonCanId);

  // Phoenix 6 StatusSignals (lecture stable)
  private final StatusSignal<Angle> m_yawSig;
  private final StatusSignal<AngularVelocity> m_yawRateSig;

  // Cache (degrés / degrés par seconde) — mis à jour 1x/cycle
  private double m_yawDeg = 0.0;
  private double m_yawRateDegPerSec = 0.0;

  // Odometry
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(0.0),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // Init signals
    m_yawSig = m_gyro.getYaw();
    m_yawRateSig = m_gyro.getAngularVelocityZWorld();

    // Fréquence d’update (important pour field-relative fluide)
    m_yawSig.setUpdateFrequency(100);
    m_yawRateSig.setUpdateFrequency(100);

    // Optimise le bus CAN (très utile)
    m_gyro.optimizeBusUtilization();

    // Optionnel : partir avec yaw = 0
    zeroHeading();
  }

  @Override
  public void periodic() {
    // Rafraîchit les signaux gyro (lecture stable)
    BaseStatusSignal.refreshAll(m_yawSig, m_yawRateSig);

    // Convertit en degrés (unit-safe)
    m_yawDeg = m_yawSig.getValue().in(Units.Degrees);
    m_yawRateDegPerSec = m_yawRateSig.getValue().in(Units.DegreesPerSecond);

    // Update odometry
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  // -------------------------
  // Gyro helpers
  // -------------------------
  private double getYawDegrees() {
    return m_yawDeg * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0.0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return heading in degrees (normalisé -180..180)
   */
  public double getHeading() {
    return Math.IEEEremainder(getYawDegrees(), 360.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return turn rate in degrees per second
   */
  public double getTurnRate() {
    return m_yawRateDegPerSec * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // -------------------------
  // Odometry
  // -------------------------
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // -------------------------
  // Drive
  // -------------------------
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Applique le scaling (si tu l’as ajouté dans Constants)
    double scale = 1.0;
    // Si tu as ajouté DriveConstants.kDriveSpeedScale, décommente :
    // scale = DriveConstants.kDriveSpeedScale;

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * scale;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * scale;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * scale;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond * scale);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }
}