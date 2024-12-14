// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class SwerveConstants {
    public static final double kMaxSpeedMeterPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static final double kFLChassisAngularOffset = -Math.PI / 2;
    public static final double kFRChassisAngularOffset = 0;
    public static final double kBLChassisAngularOffset = Math.PI;
    public static final double kBRChassisAngularOffset = Math.PI / 2;

    public static final int kFLdriveMotorID = 0;
    public static final int kFRdriveMotorID = 1;
    public static final int kBLdriveMotorID = 2;
    public static final int kBRdriveMotorID = 3;

    public static final int kFLturningMotorID = 4;
    public static final int kFRturningMotorID = 5;
    public static final int kBLturningMotorID = 6;
    public static final int kBRturningMotorID = 7;

    public static final boolean kGyroReversed = false;
  }
  
  public static final class ModuleConstants {

    public static final int kDriveMotorPinionTeeth = 14;

    public static final double kDriveMotorFreeSpdRps = NEOConstants.kFreeSpdRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDriveMotorReduction = (45.0 * 22) / (kDriveMotorPinionTeeth * 15);
    public static final double DriveWheelFreeSpdRps = (kDriveMotorFreeSpdRps * kWheelCircumferenceMeters) / kDriveMotorReduction;
  }

  public static final class OIConstants {
    public static final int kControllerPort = 0;
    public static final double kDeadBand = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpdMeterPerSecond = 3;
    public static final double kMaxAccelerationMeterPerSecondSquared = 3;
    public static final double kMaxAngularSpdRadPerSecond = Math.PI;
    public static final double kMaxAngularSpdRadPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpdRadPerSecond, kMaxAccelerationMeterPerSecondSquared);
    }

    public static final class NEOConstants {
      public static final double kFreeSpdRpm = 5676;
    }
}
