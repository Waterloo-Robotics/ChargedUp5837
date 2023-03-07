// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ArmControlState;
import frc.robot.Arm.ArmState;
import frc.robot.Arm.IntakeState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  /* Drive Base Motor Controllers */
  WPI_TalonSRX m_driveRight1 = new WPI_TalonSRX(6);
  public static WPI_TalonSRX m_driveRight2 = new WPI_TalonSRX(7);
  private final MotorControllerGroup mg_rightDrive = new MotorControllerGroup(m_driveRight1, m_driveRight2);

  WPI_TalonSRX m_driveLeft1 = new WPI_TalonSRX(8);
  public static WPI_TalonSRX m_driveLeft2 = new WPI_TalonSRX(9);
  private final MotorControllerGroup mg_leftDrive = new MotorControllerGroup(m_driveLeft1, m_driveLeft2);
  
  private final DifferentialDrive r_robotDrive = new DifferentialDrive(mg_leftDrive, mg_rightDrive);
  private final XboxController c_controller = new XboxController(1);
  Joystick bbRight = new Joystick(2);
  Joystick bbLeft = new Joystick(3);
  private final Timer timer = new Timer();

  public static SensorCollection Joint1Enc = m_driveLeft2.getSensorCollection();
  public static SensorCollection Joint2Enc = m_driveRight2.getSensorCollection();

  Arm arm = new Arm(m_driveLeft2, m_driveRight2);
  int joint1Timeout;
  int joint1TimeoutLimit = 100;
  boolean joint1TimeoutEnable = true;
  int joint2Timeout;
  int joint2TimeoutLimit = 100;
  boolean joint2TimeoutEnable = true;

  boolean timeoutOverride = false;


  public Arm.ArmState armState = Arm.ArmState.home;
  public Arm.IntakeState intakeState = IntakeState.cubeOpen;
  // ADIS16470_IMU imu = new ADIS16470_IMU();

  boolean coneControl = true;

  ArmPosition home = new ArmPosition(10, 18, 0);
  ArmPosition homeFront = new ArmPosition(16, 9, 0);
  ArmPosition homeBack = new ArmPosition(-16, 9, 0);

  /* Pickup Positions */
  ArmPosition conePickupFrontGround = new ArmPosition(34, 7, 223);
  ArmPosition conePickupBackGround = new ArmPosition(-34, 7, -223);

  ArmPosition conePickupFrontShelf = new ArmPosition(30, 5, 0);
  ArmPosition conePickupBackShelf = new ArmPosition(-30, 5, 0);

  ArmPosition cubePickupFrontGround = new ArmPosition(34, 7, 223);
  ArmPosition cubePickupBackGround = new ArmPosition(-34, 7, -223);

  ArmPosition cubePickupFrontShelf = new ArmPosition(30, 5, 0);
  ArmPosition cubePickupBackShelf = new ArmPosition(-30, 5, 0);
  
  /* Cone Scoring Positions */
  ArmPosition coneScoreFrontLow = new ArmPosition(34, 7, 223);
  ArmPosition coneScoreFrontMiddle = new ArmPosition(40, 48, 193);
  ArmPosition coneScoreFrontHigh = new ArmPosition(48, 51, 223);

  ArmPosition coneScoreBackLow = new ArmPosition(-34, 7, -223);
  ArmPosition coneScoreBackMiddle = new ArmPosition(-40, 48, -193);
  ArmPosition coneScoreBackHigh = new ArmPosition(-48, 51, -223);

  /* Cube Scoring Positions */
  ArmPosition cubeScoreFrontLow = new ArmPosition(34, 7, 223);
  ArmPosition cubeScoreFrontMiddle = new ArmPosition(33, 28, 193);
  ArmPosition cubeScoreFrontHigh = new ArmPosition(47, 39.5, 223);

  ArmPosition cubeScoreBackLow = new ArmPosition(-34, 7, -223);
  ArmPosition cubeScoreBackMiddle = new ArmPosition(-33, 28, -193);
  ArmPosition cubeScoreBackHigh = new ArmPosition(-47, 39.5, -223);

  boolean isAuto = false;
  boolean movingAuto = false;

  /* ArmPositions to store where the arm is, and where we want it to go */
  ArmPosition currentArmPosition = new ArmPosition(0, 9, 0);
  ArmPosition desiredArmPosition = new ArmPosition(0, 9, 0);

  /* Global Arm Path Planner */
  ArmPathPlanner pathPlanner = new ArmPathPlanner(homeFront, homeBack);
  /* Global Arm Sequence */
  ArmSequence currentSequence = new ArmSequence();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_driveLeft1.configOpenloopRamp(1);
    // m_driveLeft2.configOpenloopRamp(1);
    // m_driveRight1.configOpenloopRamp(1);
    // m_driveRight2.configOpenloopRamp(1);
    // imu.calibrate();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    mg_leftDrive.setInverted(true);
    Arm.mg_Joint1.setInverted(true);

    m_driveLeft1.configOpenloopRamp(0.25);
    m_driveLeft2.configOpenloopRamp(0.25);
    m_driveRight1.configOpenloopRamp(0.25);
    m_driveRight2.configOpenloopRamp(0.25);

    // Arm.m_Joint1_1.getEncoder().setInverted(true);
  }

  public void robotPeriodic() {
    SmartDashboard.putNumber("Joint 1 Angle", arm.joint1CurrentPosition());
    SmartDashboard.putNumber("Joint 2 Angle", arm.joint2CurrentPosition());


    SmartDashboard.putNumber("Joint 3 Enc", arm.joint3CurrentPosition());
    SmartDashboard.putNumber("Sequence Steps", currentSequence.getLength());
    SmartDashboard.putNumber("Current Step", currentSequence.currentIndex);
    SmartDashboard.putNumber("Joint 3 Desired Angle", Arm.joint3Angle);
    SmartDashboard.putString("Arm State", String.valueOf(armState));
    SmartDashboard.putBoolean("Brake 1", Arm.joint1Brake.get() == Value.kReverse);
    SmartDashboard.putBoolean("Brake 2", Arm.joint2Brake.get() == Value.kReverse);
    
    SmartDashboard.putBoolean("Compressor", Arm.pmc.getCompressor());
    SmartDashboard.putNumber("NEO J1 1 (A)", pdp.getCurrent(1));
    SmartDashboard.putNumber("NEO J1 2 (A)", pdp.getCurrent(2));
    SmartDashboard.putNumber("NEO J2 (A)", pdp.getCurrent(3));

    SmartDashboard.putString("Arm Ctrl State", String.valueOf(arm.armControlState));
    SmartDashboard.putBoolean("Arm in Pos", arm.isArmInPosition());
    SmartDashboard.putBoolean("Sequence Finished", currentSequence.sequenceFinished());

    SmartDashboard.putBoolean("J1 Timeout En", joint1TimeoutEnable);
    SmartDashboard.putNumber("J1 Timeout", joint1Timeout);
    SmartDashboard.putBoolean("J2 Timeout En", joint2TimeoutEnable);
    SmartDashboard.putNumber("J2 Timeout", joint2Timeout);

    arm.getArmCoordinates(arm.joint1CurrentPosition(), arm.joint2CurrentPosition());
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

  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    isAuto = false;
    currentArmPosition.setCoordinates(0, 9, 0);
    arm.joint1PID.reset();
    arm.joint2PID.reset();

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

    // Arm Position controls
      // TODO if robot drive fast for extended period of time (a few seconds?) set arm to position within bumpers

    // Uncomment for driving

  /* Claw States */

  /* Switch between Cone and Cube */
  if (bbLeft.getRawButton(9)) {
    coneControl = true;
  } else if (bbLeft.getRawButton(10)) {
    coneControl = false;
  }

  /* Home Buttons */
  if (bbLeft.getRawButton(7)) armState = ArmState.goFrontHome;
  if (bbLeft.getRawButton(8)) armState = ArmState.goBackHome;


  /* Scoring Positions */
  if (coneControl) {
    /* Claw Control */
    if (c_controller.getXButtonPressed()) intakeState = IntakeState.coneIntakeBack;
    if (c_controller.getBButtonPressed()) intakeState = IntakeState.coneIntakeForward;
    if (c_controller.getAButtonPressed()) intakeState = IntakeState.coneClosed;

    /* Pickup Positions */
    if (bbRight.getRawButton(3)) armState = ArmState.goConePickupBackGround;
    if (bbRight.getRawButton(6)) armState = ArmState.goConePickupFrontGround;

    /* Scoring Positions */
    if (bbLeft.getRawButton(1)) armState = ArmState.goConeScoreBackLow;
    if (bbLeft.getRawButton(2)) armState = ArmState.goConeScoreBackMiddle;
    if (bbLeft.getRawButton(3)) armState = ArmState.goConeScoreBackHigh;

    if (bbLeft.getRawButton(4)) armState = ArmState.goConeScoreFrontLow;
    if (bbLeft.getRawButton(5)) armState = ArmState.goConeScoreFrontMiddle;
    if (bbLeft.getRawButton(6)) armState = ArmState.goConeScoreFrontHigh;
  }
  else {
    /* Claw Control */
    if (c_controller.getXButtonPressed() || c_controller.getBButtonPressed()) intakeState = IntakeState.cubeOpen;
    if (c_controller.getAButtonPressed()) intakeState = IntakeState.cubeClosed;

    /* Pickup Positions */
    if (bbRight.getRawButton(3)) armState = ArmState.goCubePickupBackGround;
    if (bbRight.getRawButton(6)) armState = ArmState.goCubePickupFrontGround;

    /* Scoring Positions */
    if (bbLeft.getRawButton(1)) armState = ArmState.goCubeScoreBackLow;
    if (bbLeft.getRawButton(2)) armState = ArmState.goCubeScoreBackMiddle;
    if (bbLeft.getRawButton(3)) armState = ArmState.goCubeScoreBackHigh;

    if (bbLeft.getRawButton(4)) armState = ArmState.goCubeScoreFrontLow;
    if (bbLeft.getRawButton(5)) armState = ArmState.goCubeScoreFrontMiddle;
    if (bbLeft.getRawButton(6)) armState = ArmState.goCubeScoreFrontHigh;
  }

  /* Set Claw state */
  arm.updateIntake(intakeState);

  /* Enable Inverse Kinematic PID Control of arm */
  if (c_controller.getBackButtonPressed()) {
    // currentArmPosition.setCoordinates(0, 9, 0);
    currentArmPosition = new ArmPosition(home);
    armState = ArmState.home;
    isAuto = !isAuto;
  }

  /* Command arm to go to different positions */
  switch (armState) {

    case goConePickupFrontGround:
      desiredArmPosition = conePickupFrontGround;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.conePickupFrontGround;
      break;

    case goConePickupBackGround:
      desiredArmPosition = conePickupBackGround;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.conePickupBackGround;
      break;

    case goConePickupFrontShelf:
      desiredArmPosition = conePickupFrontShelf;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.conePickupFrontShelf;
      break;

    case goConePickupBackShelf:
      desiredArmPosition = conePickupBackShelf;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.conePickupBackShelf;
      break;

    case goCubePickupFrontGround:
      desiredArmPosition = cubePickupFrontGround;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubePickupFrontGround;
      break;

    case goCubePickupBackGround:
      desiredArmPosition = cubePickupBackGround;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubePickupBackGround;
      break;

    case goCubePickupFrontShelf:
      desiredArmPosition = cubePickupFrontShelf;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubePickupFrontShelf;
      break;

    case goCubePickupBackShelf:
      desiredArmPosition = cubePickupBackShelf;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubePickupBackShelf;
      break;

    case goConeScoreFrontLow:
      desiredArmPosition = coneScoreFrontLow;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreFrontLow;
      break;

    case goConeScoreFrontMiddle:
      desiredArmPosition = coneScoreFrontMiddle;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreFrontMiddle;
      break;

    case goConeScoreFrontHigh:
      desiredArmPosition = coneScoreFrontHigh;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreFrontHigh;
      break;

    case goConeScoreBackLow:
      desiredArmPosition = coneScoreBackLow;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreBackLow;
      break;

    case goConeScoreBackMiddle:
      desiredArmPosition = coneScoreBackMiddle;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreBackMiddle;
      break;

    case goConeScoreBackHigh:
      desiredArmPosition = coneScoreBackHigh;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.coneScoreBackHigh;
      break;

    case goCubeScoreFrontLow:
      desiredArmPosition = cubeScoreFrontLow;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreFrontLow;
      break;

    case goCubeScoreFrontMiddle:
      desiredArmPosition = cubeScoreFrontMiddle;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreFrontMiddle;
      break;

    case goCubeScoreFrontHigh:
      desiredArmPosition = cubeScoreFrontHigh;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreFrontHigh;
      break;

    case goCubeScoreBackLow:
      desiredArmPosition = cubeScoreBackLow;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreBackLow;
      break;

    case goCubeScoreBackMiddle:
      desiredArmPosition = cubeScoreBackMiddle;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreBackMiddle;
      break;

    case goCubeScoreBackHigh:
      desiredArmPosition = cubeScoreBackHigh;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.cubeScoreBackHigh;
      break;

    case goHome:
      desiredArmPosition = home;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.home;
      break;
    
    case goFrontHome:
      desiredArmPosition = homeFront;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.frontHome;
      break;

    case goBackHome:
      desiredArmPosition = homeBack;
      Arm.armControlState = ArmControlState.newPath;
      armState = ArmState.backHome;
      break;

    default:
    break;
  }

  /* If Inverse Kinematic PID Control is enabled */
  if (isAuto) {

    r_robotDrive.arcadeDrive(c_controller.getLeftY(), 0.75 * c_controller.getRightX());

    /*************************************************************************************
      START JOINT 3 MANUAL CONTROL
    *************************************************************************************/
    /* If manual inputs are present */
    if (Math.abs(bbRight.getY()) > 0.5 || Math.abs(bbRight.getX()) > 0.5 || 
        c_controller.getPOV() == 90 || c_controller.getPOV() == 270) 
    {
      /* Put the arm into manual mode */
      armState = ArmState.manual;
      Arm.armControlState = ArmControlState.manual;
    }

    /* Rotate Joint 3 towards back of robot */
    if (c_controller.getPOV() == 90) {
      currentArmPosition.incrementZ(-2);
    }
    /* Rotate Joint 3 towards front of robot */
    else if (c_controller.getPOV() == 270) {
      currentArmPosition.incrementZ(2);
    }

    /* If Joint 3 angle is over 225 degrees reset to 225*/
    if (currentArmPosition.z > 225) {
      currentArmPosition.z = 225;
    }
    /* If Joint 3 angle is under -225 degrees reset to -225*/
    else if (currentArmPosition.z < -225) {
      currentArmPosition.z = -225;
    }

    if (bbRight.getRawButton(5)) currentArmPosition.z = 0;

    /* Button to reset Joint 3 encoder */
    if (bbRight.getRawButtonPressed(1)) {
      Arm.m_Joint3.getEncoder().setPosition(0);
    }
    /*************************************************************************************
      END JOINT 3 MANUAL CONTROL
    *************************************************************************************/

    /*************************************************************************************
      START X & Y MANUAL CONTROL
    *************************************************************************************/
    /* Increment X & Y */
    currentArmPosition.incrementX(deadZone(bbRight.getX()) * 0.25);
    currentArmPosition.incrementY(deadZone(bbRight.getY()) * -0.25);

    if (armState == ArmState.manual) {
      /* If current position is within frame perimeter */
      if (currentArmPosition.returnSide() == 0) {
        /* Force Y to be at 9 in to avoid eratic movement*/
        currentArmPosition.y = 9;
        // TODO: Remove setting Joint 3 to 0 once Arm Path Planning is working
        currentArmPosition.z = 0;
      }
    }
    
    /* Update SmartDashboard with attempted X & Y */
    SmartDashboard.putBoolean("Is Valid", arm.isArmPositionValid(currentArmPosition.x, currentArmPosition.y));
    SmartDashboard.putNumber("Attempted x", currentArmPosition.x);
    SmartDashboard.putNumber("Attempted y", currentArmPosition.y);

    SmartDashboard.putBoolean("Check 1", (Arm.c <= (Arm.a + Arm.b)));
    SmartDashboard.putBoolean("Check 2", (Arm.c >= (Arm.b - Arm.a)));
    SmartDashboard.putBoolean("Check 3", ((Math.toDegrees(Arm.joint1Angle) <= 60) && (Math.toDegrees(Arm.joint1Angle) >= -60)));
    SmartDashboard.putBoolean("Check 4", ((Math.toDegrees(Arm.joint2Angle) <= 150) && (Math.toDegrees(Arm.joint2Angle) >= -150)));
    SmartDashboard.putBoolean("Check 5", (currentArmPosition.y <= 70.5));
    SmartDashboard.putBoolean("Check 6", (currentArmPosition.x <= 63.0 && currentArmPosition.x >= -63.0));

    /* If attempted arm position is invalid during manual control */
    if (!arm.isArmPositionValid(currentArmPosition.x, currentArmPosition.y) && armState == ArmState.manual) {
      /* Reset currentArmPosition to last know valid position */
      currentArmPosition.x = Arm.lastValidX;
      currentArmPosition.y = Arm.lastValidY;
    } 
    /* If attempted arm position is valid, update last know valid position */
    else if (armState == ArmState.manual) {
      Arm.lastValidX = currentArmPosition.x;
      Arm.lastValidY = currentArmPosition.y;
    }

    /* Update Smart Dashboard with valid X & Y */
    SmartDashboard.putNumber("Valid x", currentArmPosition.x);
    SmartDashboard.putNumber("Valid y", currentArmPosition.y);
    /*************************************************************************************
      END X & Y MANUAL CONTROL
    *************************************************************************************/
    
    /*************************************************************************************
      START ARM STATE CONTROL
    *************************************************************************************/
    if (Arm.armControlState == ArmControlState.manual) {
      /* If arm is controller manually, the current position is updated above, the arm
         will be told to run to updated position in the arm.updateArm() call below. 
        */
    } 
    /* If a new desiredArmPositon has been requested */
    else if (Arm.armControlState == ArmControlState.newPath) {

      /* Put arm in runningPath state */
      Arm.armControlState = ArmControlState.runningPath;
      /* Create a new sequence to move from currentArmPosition to desiredArmPosition */
      currentSequence = pathPlanner.planPath(new ArmPosition(currentArmPosition), new ArmPosition(desiredArmPosition));

      joint1TimeoutEnable = true;
      joint2TimeoutEnable = true;
      timeoutOverride = false;
    }

    /* If the arm is in the middle of a sequence */ 
    else if (Arm.armControlState == ArmControlState.runningPath) {

      /* Next position and path finished */
      if ( (arm.isArmInPosition() || timeoutOverride) && !currentSequence.sequenceFinished()) {
        timeoutOverride = false;
        /* set currentArmPosition to next position in the sequence */
        currentArmPosition = currentSequence.nextPosition();
        joint1TimeoutEnable = true;
        joint2TimeoutEnable = true;
      } 
      /* If the arm is in position and the sequence is finished */
      else if (arm.isArmInPosition() && currentSequence.sequenceFinished()) {
        /* Put the arm in a completed path state */
        Arm.armControlState = ArmControlState.completedPath;
      }

      /* Arm position timeouts */
      if (!arm.isJoint1AtPosition() && joint1TimeoutEnable && joint1Timeout <= joint1TimeoutLimit) {
        joint1Timeout++;
      } 

      if (!arm.isJoint2AtPosition() && joint2TimeoutEnable && joint2Timeout <= joint2TimeoutLimit) {
        joint2Timeout++;
      } 

      /* Service timeouts */
      if ( (joint1Timeout > joint1TimeoutLimit || arm.isJoint1AtPosition()) && 
           (joint2Timeout > joint2TimeoutLimit || arm.isJoint2AtPosition()) ) {
        timeoutOverride = true;
        
        /* Reset timeouts */
        joint1Timeout = 0;
        joint1TimeoutEnable = false;
        joint2Timeout = 0;
        joint2TimeoutEnable = false;
      }
    }
    
    /* Update Arm PID calculation and motor set speeds */
    arm.updateArm(currentArmPosition);

    /*************************************************************************************
      END ARM STATE CONTROL
    *************************************************************************************/
  }
  /* Inverse Kinematic PID Control is disabled */
  else {

    r_robotDrive.arcadeDrive(0, 0);

    Arm.joint1Brake.set(Value.kForward);
    Arm.joint2Brake.set(Value.kForward);
    
    arm.setJoint1(deadZone(c_controller.getLeftX()) * 0.1);
    arm.setJoint2(deadZone(c_controller.getRightX()) * 0.075);

  }

  SmartDashboard.putBoolean("Is joint 1 at position", arm.isJoint1AtPosition());
  SmartDashboard.putBoolean("Is joint 2 at position", arm.isJoint2AtPosition());
  SmartDashboard.putBoolean("Auto Status", isAuto);
    
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /* Apply a fixed dead zone to an input */
  public double deadZone(double input) {
    // TODO: Fix to scale input after deadzone
    
    if (input < 0.1 && input > -0.1) return 0;
    else return input;

  }

}
