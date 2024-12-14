// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class SwerveModule extends SubsystemBase {
  
  private final SparkMax driveMotor;
  private final SparkFlex turningMotor;
  
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkClosedLoopController m_driveClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(int driveMotorID, int turningMotorID, double chassisAngularOffset) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turningMotor = new SparkFlex(turningMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder();

    m_driveClosedLoopController = driveMotor.getClosedLoopController();
    m_turningClosedLoopController = turningMotor.getClosedLoopController();

    driveMotor.configure(Configs.SwerveModule.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turningMotor.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    driveEncoder.setPosition(0);
  } 
    
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
    new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(m_chassisAngularOffset));

    correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

    m_driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

    m_desiredState = desiredState;
  }

  public void resetEncoder() {
    driveEncoder.setPosition(0);
  }
}
