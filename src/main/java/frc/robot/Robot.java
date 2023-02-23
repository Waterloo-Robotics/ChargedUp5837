// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

// import java.beans.Encoder;

// import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
// import com.revrobotics.EncoderType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  /* Drive Base Motor Controllers */
  WPI_TalonSRX m_driveRight1 = new WPI_TalonSRX(6);
  WPI_TalonSRX m_driveRight2 = new WPI_TalonSRX(7);
  private final MotorControllerGroup mg_rightDrive = new MotorControllerGroup(m_driveRight1, m_driveRight2);

  WPI_TalonSRX m_driveLeft1 = new WPI_TalonSRX(8);
  WPI_TalonSRX m_driveLeft2 = new WPI_TalonSRX(9);
  private final MotorControllerGroup mg_leftDrive = new MotorControllerGroup(m_driveLeft1, m_driveLeft2);

  // Encoder joint1Enc = new Encoder(0, 1);
  // Encoder joint2Enc = new Encoder(2, 3);
  // CANSparkMax neo = new CANSparkMax(4, MotorType.kBrushless);
  // SparkMaxAbsoluteEncoder encoder2 = neo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  // RelativeEncoder encoder = neo.getEncoder();
  
  
  private final DifferentialDrive r_robotDrive = new DifferentialDrive(mg_leftDrive, mg_rightDrive);
  private final XboxController c_controller = new XboxController(1);
  Joystick bbRight = new Joystick(0);
  Joystick bbLeft = new Joystick(3);
  private final Timer timer = new Timer();

  public Arm.ArmState armState = Arm.ArmState.home;
  // ADIS16470_IMU imu = new ADIS16470_IMU();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // imu.calibrate();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    mg_leftDrive.setInverted(true);
    mg_Joint1.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    // neo.getEncoder().setPosition(0);
    // while (neo.getEncoder().getPosition() < 100) {
      // Drive forwards half speed, make sure to turn input squaring off
      // m_robotDrive.arcadeDrive(-1, 0.0, false);
      // neo.set(1);
      // System.out.println(neo.getEncoder().getPosition());
      // System.out.println(neo.getEncoder().getVelocity());
    // }

    // while (!m_controller.getRawButton(1)) {

      // neo.set(0);
      // System.out.println(neo.getEncoder().getPosition());

    // }

    // while (neo.getEncoder().getPosition() > 0) {

      // neo.set(-1);
      // System.out.println(neo.getEncoder().getPosition());
    // }

    // while (!m_controller.getRawButton(1)) {

      // neo.set(0);
      // System.out.println(neo.getEncoder().getPosition());

    // }

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    // neo.getEncoder().setPosition(0);
    // imu.setYawAxis(ADIS16470_IMU.IMUAxis.kY);

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    /** CONTROLS:
     * Driver 1 Closes and opens claw, Driver one controls what side of the claw is used for pickup
     * Driver 1 (and 2?) controls joint 3
     *
     * Driver 2 controls arm automatic positioning and manual adjustments (all through inverse kinematics)
     * Driver 2 controls initial joint 3 position
     * */

    // get drive joysticks

    // Dead Zone calculations

    // Update Drivebase

    // Claw Control
      // rotate claw
      // Actuate pistons

    // Arm Position controls
      // TODO if robot drive fast for extended period of time (a few seconds?) set arm to position within bumpers

    // Uncomment for driving
//    m_robotDrive.arcadeDrive(m_controller.getLeftY(), 1 * m_controller.getRightX());

//    if (m_controller.getAButton()) armState = ArmState.conePickup;
//    if (m_controller.getBButton()) armState = ArmState.coneScoreLow;

//    this.updateArm();


    // nick arm stuff
    //  double y1Math, y2Math;

    //  double y1Axis = m_controller.getLeftY();
    //  double y2Axis = m_controller.getRightY();


    //  if (Math.abs(y1Axis) < 0.25){
    //    y1Math = 0.0;
    //  }
    //  else {
    //      y1Math = Math.signum(y1Axis)*(Math.abs(y1Axis) - .25)*1.333;
    //  }

    //  if (Math.abs(y2Axis) < 0.25){
    //    y2Math = 0.0;
    //  }
    //  else {
    //    y2Math = Math.signum(y2Axis)*(Math.abs(y2Axis) - .25)*1.333;
    //  }

    //  y1Math = y1Math * 0.25;
    //  y2Math = y2Math * 0.25;
    if (c_controller.getLeftX() > -0.1 && c_controller.getLeftX() < 0.1) {

      leftXMath = 0;

    } else leftXMath = c_controller.getLeftX() * 0.1;

//    setJoint1(leftXMath);

//    if (c_controller.getAButtonPressed()) m_Joint1_1.getEncoder().setPosition(0);

//    System.out.println("Joint 1 NEO enc: " + m_Joint1_1.getEncoder().getPosition());

    // Arm Testing
//     m_Joint1.set(y2Math);
//     m_Joint2.set(y1Math);

//     SmartDashboard.putNumber("Arm I", y2Math);
//     SmartDashboard.putNumber("Arm D", y1Math);



    // encoder2 = neo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // neo.set(m_controller.getY());

    // if (m_controller.getRawButton(1)) neo.getEncoder().setPosition(0);

//    System.out.println("Joint 1 Positive Limit: " + ls_joint1PosLimit.get());
//    System.out.println("Joint 1 Negative Limit: " + ls_joint1NegLimit.get());
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  double leftXMath = 0;
  @Override
  public void testPeriodic() {

    // if (bbRight.getRawButton(1)) setArmCoordinates(36, 15);
    

  }

}
