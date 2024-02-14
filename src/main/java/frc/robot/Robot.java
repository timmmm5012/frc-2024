// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

// /**
//  * The VM is configured to automatically run this class, and to call the
//  * functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the
//  * name of this class or
//  * the package after creating this project, you must also update the manifest
//  * file in the resource
//  * directory.
//  */
public class Robot extends TimedRobot {

  private final CANSparkMax m_leftDrive = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax m_rightDrive = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax follow_leftDrive = new CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax shooter_motor = new CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax follow_rightDrive = new CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax intake_upper = new CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushed);
  private final CANSparkMax intake_lower = new CANSparkMax(7, CANSparkLowLevel.MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

//   /**
//    * This function is run when the robot is first started up and should be used
//    * for any
//    * initialization code.
//    */
//   @Override
  public void robotInit() {
    follow_leftDrive.follow(m_leftDrive);
    follow_rightDrive.follow(m_rightDrive);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getLeftX());
    shooter_motor.set(m_controller.getRightY());
    if (m_controller.getAButton()) {
      Float intake_motor_speed = 0.25f;
      intake_upper.set(intake_motor_speed / 2);
      intake_lower.set(intake_motor_speed);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_robotDrive.arcadeDrive(1, -1);
  }
}
