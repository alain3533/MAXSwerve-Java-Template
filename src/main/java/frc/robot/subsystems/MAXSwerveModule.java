package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0.0;

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean driveInverted) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // ✅ Choix config drive selon inversion
    var driveCfg = driveInverted
        ? Configs.MAXSwerveModule.drivingConfigInverted
        : Configs.MAXSwerveModule.drivingConfig;

    m_drivingSpark.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_drivingEncoder.setPosition(0.0);
  }

  // ✅ Compat avec ancien code (si jamais)
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this(drivingCANId, turningCANId, chassisAngularOffset, false);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState corrected = new SwerveModuleState();
    corrected.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    corrected.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getPosition());

    // ✅ Pas de warning (WPILib 2026)
    corrected.optimize(currentAngle);

    m_drivingClosedLoopController.setSetpoint(corrected.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(corrected.angle.getRadians(), ControlType.kPosition);
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0.0);
  }
}