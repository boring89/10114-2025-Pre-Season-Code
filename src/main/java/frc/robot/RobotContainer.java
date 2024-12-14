// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SwerveSubsystem m_drive = new SwerveSubsystem();

    private XboxController Xinput = new XboxController(Constants.OIConstants.kControllerPort);

    public RobotContainer() {     
        configurButtonBinding();

        m_drive.setDefaultCommand(
            new RunCommand(
                () -> m_drive.drive(
                    MathUtil.applyDeadband(Xinput.getLeftY(), Constants.OIConstants.kDeadBand), 
                    MathUtil.applyDeadband(Xinput.getLeftX(), Constants.OIConstants.kDeadBand), 
                    MathUtil.applyDeadband(Xinput.getRightX(), Constants.OIConstants.kDeadBand), 
                    true), 
                    m_drive));
    }

    private void configurButtonBinding() {
        new JoystickButton(Xinput, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_drive.setX(), m_drive) );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}


