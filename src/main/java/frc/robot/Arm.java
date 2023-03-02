package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Arm {

    /* Arm Motor Controllers */
    static CANSparkMax m_Joint1_1 = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    static CANSparkMax m_Joint1_2 = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    static CANSparkMax m_Joint2 = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    static MotorControllerGroup mg_Joint1 = new MotorControllerGroup(m_Joint1_1, m_Joint1_2);

    static CANSparkMax m_Joint3 = new CANSparkMax(13, MotorType.kBrushless);

    // Limit Switches for joint 1
    DigitalInput ls_joint1PosLimit = new DigitalInput(0);
    DigitalInput ls_joint1NegLimit = new DigitalInput(1);

    WPI_TalonSRX joint2EncoderTalon;
    WPI_TalonSRX joint1EncoderTalon;

    DutyCycleEncoder joint1Encoder = new DutyCycleEncoder(2);

    AnalogEncoder joint1AnalogEncoder = new AnalogEncoder(0);

    PneumaticsControlModule pmc = new PneumaticsControlModule(1);
    public static DoubleSolenoid joint1Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
    public static DoubleSolenoid joint2Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3);
    public static DoubleSolenoid intake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5);
    public static DoubleSolenoid coneSwitch = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 6, 7);

    public enum ArmState {
        /* Cone Pickups */
        goConePickupFrontGround, conePickupFrontGround,
        goConePickupBackGround, conePickupBackGround,
        goConePickupFrontShelf, conePickupFrontShelf,
        goConePickupBackShelf , conePickupBackShelf,
        /* Cube Pickups */
        goCubePickupFrontGround , cubePickupFrontGround,
        goCubePickupBackGround , cubePickupBackGround,
        goCubePickupFrontShelf , cubePickupFrontShelf,
        goCubePickupBackShelf , cubePickupBackShelf,
        /* Cone Score */
        goConeScoreFrontLow, coneScoreFrontLow,
        goConeScoreFrontMiddle, coneScoreFrontMiddle,
        goConeScoreFrontHigh, coneScoreFrontHigh,
        goConeScoreBackLow, coneScoreBackLow,
        goConeScoreBackMiddle, coneScoreBackMiddle,
        goConeScoreBackHigh, coneScoreBackHigh,
        /* Cube Score */
        goCubeScoreFrontLow, cubeScoreFrontLow,
        goCubeScoreFrontMiddle, cubeScoreFrontMiddle,
        goCubeScoreFrontHigh, cubeScoreFrontHigh,
        goCubeScoreBackLow, cubeScoreBackLow,
        goCubeScoreBackMiddle, cubeScoreBackMiddle,
        goCubeScoreBackHigh, cubeScoreBackHigh,
        /* Homes */
        goFrontHome, frontHome, 
        goBackHome, backHome, 
        goHome, home, 
        manual

    }

    public enum ArmControlState {

        manual,
        newPath,
        runningPath,
        completedPath

    }

    public static ArmControlState armControlState = ArmControlState.manual;

    int numArmStates = 12;

    double[][] armSetPositions = new double[numArmStates][3];

    public enum IntakeState {

        coneIntakeForward, coneIntakeBack,
        cubeOpen, cubeClosed,
        coneClosed

    }

    double joint1kP = 0.0040;
    double joint1kI = 0.0026;
    double joint1kD = 0.0027;
    PIDController joint1PID = new PIDController(joint1kP, joint1kI, joint1kD);

    double joint2kP = 0.0050;
    double joint2kI = 0.0010;
    double joint2kD = 0.0035;
    PIDController joint2PID = new PIDController(joint2kP, joint2kI, joint2kD);

    double joint3kP = 0.002;
    double joint3kI = 0;
    double joint3kD = 0;
    PIDController joint3PID = new PIDController(joint2kP, joint2kI, joint2kD);

    public Arm(WPI_TalonSRX joint1EncoderTalon, WPI_TalonSRX joint2EncoderTalon) {
        
        /* Joint 1 and 2 Encoders */
        this.joint1EncoderTalon = joint1EncoderTalon;
        this.joint2EncoderTalon = joint2EncoderTalon;

        joint1Encoder.setDutyCycleRange(0, 1);
        joint1Encoder.setDistancePerRotation(360);
        joint1Encoder.setPositionOffset(0.0);

        joint1AnalogEncoder.setDistancePerRotation(360);
        /* Joint 1 Ramp Rate */
        m_Joint1_1.setOpenLoopRampRate(3);
        m_Joint1_2.setOpenLoopRampRate(3);
        /* Joint 2 and 3 Ramp Rate */
        m_Joint2.setOpenLoopRampRate(3);
        m_Joint3.setOpenLoopRampRate(3);

        /* PID Tolerance */
        joint1PID.setTolerance(5);
        joint2PID.setTolerance(5);
        joint3PID.setTolerance(5);

    }

    public static boolean isHomed = false;

    int joint1AtPositionPersistence, joint2AtPositionPersistence;

    public static double joint1Angle, joint2Angle = 0;
    public static double c = 0;
    public static double angleY, angleA = 0;

    public static double a = 33;
    public static double b = 42;

    public static double joint2Calc;

    public void setArmCoordinates(ArmPosition position) {

        setArmCoordinates(position.x, position.y);

    }

    public void setArmCoordinates(double x, double y) {

        c = Math.sqrt((x * x) + (y * y));
        angleY = Math.asin(y/c);
        double ACalc = ((b * b) + (c * c) - (a * a)) / (2 * c * b); // calculate cosine angle A
        angleA = Math.acos(ACalc); // Find angle A
        joint1Angle = (Math.PI / 2) - (angleY + angleA);

        double cosineJoint2 = ((a * a) + (b * b) - (c * c)) / (2 * a * b); // calculate cosine Joint2 or angle C
        joint2Angle = Math.acos(cosineJoint2);

        if ((x < 0.5 && x > -0.5) && (y < 9.5)) {

            joint1Angle = 0;
            joint2Angle = 0;

        }

        if (x < 0) {

            joint1Angle = -joint1Angle;
            joint2Angle = -joint2Angle;

        }

        // System.out.println("Joint 1 Angle: " + Math.toDegrees(joint1Angle));

        // System.out.println("Joint 2 Angle: " + Math.toDegrees(joint2Angle));

        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));

    }

    public boolean isArmPositionValid(double x, double y) {

        setArmCoordinates(x, y);

        return 
        (c <= (a + b)) && // not longer than physically possible
        (c >= (b - a)) && // not shorter than physically possible
        ((Math.toDegrees(joint1Angle) <= 60) && (Math.toDegrees(joint1Angle) >= -60)) && // joint 1 isn't outside of limits
        ((Math.toDegrees(joint2Angle) <= 120) && (Math.toDegrees(joint2Angle) >= -120)) && // joint 2 isn't outside of limits
        (y <= 70.5) && // doesn't exceed height limit
        (x <= 63.0 && x >= -63.0); // doesn't exceed 48 inches from frame

    }

    // double x, y = 0;
    double joint1Pos, joint2Pos, joint2SetPoint = 0;
    double joint1Speed, joint2Speed = 0;
    public static double lastValidX = 0;
    public static double lastValidY = 0;

    public void updateArm(ArmPosition position) {

        double x = position.x;
        double y = position.y;
        double z = position.z;

        updateArm(x, y, z);

    }
    public void updateArm(double x, double y, double z) {

        if (isArmPositionValid(x, y)) {

            setArmCoordinates(x, y);
            if ((x < 0.5 && x > -0.5) && y < 9.5) {

                joint1Angle = 0;
                joint2Angle = 0;

            }

            joint2SetPoint = joint2Angle - joint1Angle;
            joint1PID.setSetpoint(Math.toDegrees(joint1Angle));
            joint2PID.setSetpoint(Math.toDegrees(joint2SetPoint));

            joint1Speed = joint1PID.calculate(joint1CurrentPosition());
            joint2Speed = joint2PID.calculate(joint2CurrentPosition());

            dynamicI(joint1PID.getPositionError(), joint2PID.getPositionError());
            joint3AutoTest(z);

            if (!isJoint1AtPosition()) {
                if (isHomed) setJoint1(joint1Speed);
                joint1Brake.set(DoubleSolenoid.Value.kForward);
                joint1AtPositionPersistence = 0;
            } else {

                
                if (joint1AtPositionPersistence > 25) {
                    joint1Brake.set(DoubleSolenoid.Value.kReverse);
                } else {

                    joint1AtPositionPersistence++;

                }

            }

            if (!isJoint2AtPosition()) {
                if (isHomed) setJoint2(joint2Speed);
                joint2Brake.set(DoubleSolenoid.Value.kForward);
                joint2AtPositionPersistence = 0;
            } else {

                if (joint2AtPositionPersistence > 50) {
                    joint2Brake.set(DoubleSolenoid.Value.kReverse);
                } else {

                    joint2AtPositionPersistence++;

                }

            }
        
        } else {}

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));
        SmartDashboard.putNumber("joint2Setpoint", Math.toDegrees(joint2SetPoint));
        SmartDashboard.putNumber("Joint 1 Power", mg_Joint1.get());
        SmartDashboard.putNumber("Joint 2 Power", m_Joint2.get());
        SmartDashboard.putNumber("Joint 1 Error", joint1PID.getPositionError());
        SmartDashboard.putNumber("Joint 2 Error", joint2PID.getPositionError());

        //  joint1Pos = joint1Enc.get() * 2 * Math.PI;
//    joint2Pos = joint2Enc.get() * 2 * Math.PI;

//     m_Joint2.set(joint2Speed);

    }

    double joint3Speed = 0;

    public void dynamicI(double joint1Error, double joint2Error) {

        if (Math.abs(joint1Error) < 5) {

            joint1PID.setI(0);


        } else if (Math.abs(joint1Error) < 10) {

            joint1PID.setI(joint1kI);

        } else if (Math.abs(joint1Error) < 17) {

            joint1PID.setI(joint1kI * 0.45);

        } else {

            joint1PID.setI(0);

        }

        if (Math.abs(joint2Error) < 5) {

            joint2PID.setI(0);

        } else if (Math.abs(joint2Error) < 15) {

            joint2PID.setI(joint2kI);

        } else if (Math.abs(joint2Error) < 20) {

            // joint2PID.setI(joint2kI * 0.4);
            joint2PID.setI(0);

        } else {

            joint2PID.setI(0);

        }

    }

    public static double joint3Angle;

    public void joint3AutoTest(double angle) {

        joint3Angle = angle;
        joint3PID.setSetpoint(angle);
        joint3Speed = joint3PID.calculate(joint3CurrentPosition());
        setJoint3(joint3Speed);

    }

    public void setJoint1(double power) {

        if ((power > 0 && !ls_joint1PosLimit.get()) || (power < 0 && !ls_joint1NegLimit.get())) {

            joint1Speed = 0;

        } else if (power > 0.3) {

            joint1Speed = 0.3;

        } else if (power < -0.3) {

            joint1Speed = -0.3;

        } else {

            joint1Speed = power;

        }

        if (joint1Brake.get() == Value.kReverse)
        { joint1Speed = 0; }

        mg_Joint1.set(joint1Speed);
    }

    public void setJoint2(double power) {

        if (power > 0.25) {

            joint2Speed = 0.25;

        } else if (power < -0.25) {

            joint2Speed = -0.25;

        } else {

            joint2Speed = power;

        }

        if (joint2Brake.get() == Value.kReverse)
        { joint2Speed = 0; }

        m_Joint2.set(joint2Speed);
    }

    public void setJoint3(double power) {

        if (power > 0.7) {

            joint3Speed = 0.7;

        } else if (power < -0.7) {

            joint3Speed = -0.7;

        } else {

            joint3Speed = power;

        }

        m_Joint3.set(joint3Speed);
        // SmartDashboard.putNumber("Joint 2 Angle", joint2CurrentPosition());

    }

    public double joint1CurrentPosition() {

        // return ((Robot.Joint2Enc.getPulseWidthPosition() - 351.0) / 4096.0 * 360.0);
        return Robot.Joint1Enc.getPulseWidthPosition() / 11.37 - 223;

    }

    public double joint2CurrentPosition() {

        // return ((Robot.Joint2Enc.getPulseWidthPosition() - 351.0) / 4096.0 * 360.0);
        return Robot.Joint2Enc.getPulseWidthPosition() / 11.37 - 268;

    }

    public double joint3CurrentPosition() {

        return (m_Joint3.getEncoder().getPosition()) / 60 * 200.0 * 1.125;

    }

    public boolean isJoint1AtPosition() {

        return (Math.abs(joint1PID.getPositionError()) < 5);

    }

    public boolean isJoint2AtPosition() {

        return (Math.abs(joint2PID.getPositionError()) < 5);

    }

    public boolean isJoint3AtPosition() {

        return (Math.abs(joint3PID.getPositionError()) < 5);

    }

    public boolean isArmInPosition() {

        return isJoint1AtPosition() && isJoint2AtPosition() && isJoint3AtPosition();

    }

    public static boolean isHomeActive = false;

    public void homeJoint1(boolean button) {

        if (button) isHomeActive = true;

        if (isHomeActive) setJoint1(0.05);
        joint2PID.setSetpoint(0);
        joint2Speed = joint2PID.calculate(joint2CurrentPosition());
        setJoint2(joint2Speed);

        if (m_Joint1_1.getEncoder().getVelocity() < -0.01 && !ls_joint1PosLimit.get()) {
        
            m_Joint1_1.getEncoder().setPosition(24.59508514404297 / -2.0);
            isHomeActive = false;
            isHomed = true;

        }

        // System.out.println("Velocity: " + m_Joint1_1.getEncoder().getVelocity());
        
    }

    public void updateIntake(IntakeState intakeState) {

        switch (intakeState) {

            case cubeOpen:
            intake.set(Value.kForward);
            coneSwitch.set(Value.kForward);
            break;

            case coneIntakeForward:
            intake.set(Value.kReverse);
            coneSwitch.set(Value.kForward);
            break;

            case coneIntakeBack:
            intake.set(Value.kReverse);
            coneSwitch.set(Value.kReverse);
            break;

            case cubeClosed:
            intake.set(Value.kReverse);
            break;
            
            case coneClosed:
            intake.set(Value.kForward);
            coneSwitch.set(Value.kForward);
            break;

        }

    }

}
