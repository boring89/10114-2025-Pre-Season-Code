package frc.robot.subsystems;

import java.lang.reflect.Field;

import org.opencv.core.Mat;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule FL = new SwerveModule(
        SwerveConstants.kFLdriveMotorID, 
        SwerveConstants.kFLturningMotorID, 
        SwerveConstants.kFLChassisAngularOffset);
    private final SwerveModule FR = new SwerveModule(
        SwerveConstants.kFRdriveMotorID, 
        SwerveConstants.kFRturningMotorID, 
        SwerveConstants.kFRChassisAngularOffset);
    private final SwerveModule BL = new SwerveModule(
        SwerveConstants.kBLdriveMotorID, 
        SwerveConstants.kBLturningMotorID, 
        SwerveConstants.kBLChassisAngularOffset);
    private final SwerveModule BR = new SwerveModule(
        SwerveConstants.kBRdriveMotorID, 
        SwerveConstants.kBRturningMotorID, 
        SwerveConstants.kBRChassisAngularOffset);

    private AHRS gyro;

    SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
        SwerveConstants.kSwerveKinematics, 
        gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        });

        public SwerveSubsystem() {   
        }

        @Override
        public void periodic() {
            m_Odometry.update(
                gyro.getRotation2d(), 
                new SwerveModulePosition[] {
                    FL.getPosition(),
                    FR.getPosition(),
                    BL.getPosition(),
                    BR.getPosition()
                });
        }

        public Pose2d getPose() {
            return m_Odometry.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose) {
            m_Odometry.resetPosition(gyro.getRotation2d(), 
            new SwerveModulePosition[] {
                FL.getPosition(), 
                FR.getPosition(), 
                BL.getPosition(), 
                BR.getPosition()
            }, 
            pose);
        }

        public void drive(double xSpeed, double ySpeed, double rot, boolean FieldRelative) {
            double xSpeedDelivered = xSpeed * SwerveConstants.kMaxSpeedMeterPerSecond;
            double ySpeedDelivered = ySpeed * SwerveConstants.kMaxSpeedMeterPerSecond;
            double rotSpeedDelivered = rot * SwerveConstants.kMaxAngularSpeed;

            var SwerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                FieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, 
                    gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered));
                SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, 
                SwerveConstants.kMaxSpeedMeterPerSecond);
                FL.setDesiredState(SwerveModuleStates[0]);
                FR.setDesiredState(SwerveModuleStates[1]);
                BL.setDesiredState(SwerveModuleStates[2]);
                BR.setDesiredState(SwerveModuleStates[3]);
        }

        public void setX() {
            FL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            FR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            BL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            BR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        }

        public void setModuleStates(SwerveModuleState[] desiredState) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 
            SwerveConstants.kMaxSpeedMeterPerSecond);
            FL.setDesiredState(desiredState[0]);
            FR.setDesiredState(desiredState[1]);
            BL.setDesiredState(desiredState[2]);
            BR.setDesiredState(desiredState[3]);
        }

        public void resetEncoders() {
            FL.resetEncoder();
            FR.resetEncoder();
            BL.resetEncoder();
            BR.resetEncoder();
        }

        public void zeroHeading() {
            gyro.reset();
        }

        public double getHeading() {
            return Math.IEEEremainder(gyro.getAngle(), 360);
        }

        public double getTurnRate() {
            return gyro.getRate() * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
        }
}
