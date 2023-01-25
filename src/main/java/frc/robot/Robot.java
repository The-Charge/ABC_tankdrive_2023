// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private WPI_TalonSRX leftFrontMotor;
  private WPI_TalonSRX leftRearMotor;

  // private MotorControllerGroup left;
  private WPI_TalonSRX rightFrontMotor;
  private WPI_TalonSRX rightRearMotor;
  // private MotorControllerGroup right;

  // private final MotorController leftFrontMotor = new WPI_TalonSRX(1);
  // private final MotorController righFronttMotor = new WPI_TalonSRX(7);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    leftFrontMotor = new WPI_TalonSRX(1);
    rightFrontMotor = new WPI_TalonSRX(7);
    leftRearMotor = new WPI_TalonSRX(2);
    rightRearMotor = new WPI_TalonSRX(8);

    rightFrontMotor.setInverted(true);

    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    m_myRobot = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
