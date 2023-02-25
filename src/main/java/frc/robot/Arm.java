package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

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

    PneumaticsControlModule pmc = new PneumaticsControlModule(1);
    DoubleSolenoid joint1Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
    DoubleSolenoid joint2Brake = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 3, 2);

    public enum ArmState {

        conePickup, cubePickup,
        coneScoreGround, cubeScoreGround,
        coneScoreLow, cubeScoreLow,
        coneScoreHigh, cubeScoreHigh,
        home, 
        manual

    }

    double joint1kP = 0.004;
    double joint1kI = 0;
    double joint1kD = 0;
    PIDController joint1PID = new PIDController(joint1kP, joint1kI, joint1kD);

    double joint2kP = 0.004;
    double joint2kI = 0;
    double joint2kD = 0;
    PIDController joint2PID = new PIDController(joint2kP, joint2kI, joint2kD);

    public Arm(WPI_TalonSRX joint2EncoderTalon) {
        this.joint2EncoderTalon = joint2EncoderTalon;
    }

    public static boolean isHomed = false;

    // Set Arm Position(x, y) return boolean isValidPosition

    // Set Arm Position(ArmState armState) goes to preset position

    // Get Arm Position return currentPosition[x, y]

    // Set Claw State(cone or cube, right or left, open or closed) go to preset joint 3 and pneumatics

    double joint1Angle, joint2Angle = 0;
    double c = 0;
    double angleY, angleA = 0;

    double a = 33;
    double b = 42;

    double joint2Calc;

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
    double joint1Pos, joint2Pos = 0;
    double joint1Speed, joint2Speed = 0;
    public static double lastValidX = 0;
    public static double lastValidY = 0;
    public void updateArm(double x, double y) {

        // switch (armState) {

        //     case conePickup:
        //         x = 30;
        //         y = 5;
        //         break;

        //     case coneScoreLow:
        //         x = 28;
        //         y = 38;
        //         break;

        //     case home:
        //         x = 0;
        //         y = 9;
        //         // joint1Angle = 0;
        //         // joint2Angle = 0;
        //         break;

        // }
        /*if (armState != ArmState.home)*/ 
        if (isArmPositionValid(x, y)) {

            setArmCoordinates(x, y);
            if (x == 0 && y == 9) {

                joint1Angle = 0;
                joint2Angle = 0;

            }
            joint1PID.setSetpoint(Math.toDegrees(joint1Angle));
            joint2PID.setSetpoint(Math.toDegrees(joint2Angle));

            joint1Speed = joint1PID.calculate(joint1CurrentPosition());
            joint2Speed = joint2PID.calculate(joint2CurrentPosition());

            if (!isJoint1AtPosition()) {
                if (isHomed) setJoint1(joint1Speed);
                joint1Brake.set(DoubleSolenoid.Value.kForward);
            } else {

                joint1Brake.set(DoubleSolenoid.Value.kReverse);
                setJoint1(0);

            }

            if (!isJoint2AtPosition()) {
                if (isHomed) setJoint2(joint2Speed);
                joint2Brake.set(DoubleSolenoid.Value.kForward);
            } else {

                joint2Brake.set(DoubleSolenoid.Value.kReverse);
                setJoint2(0);

            }
        
        } else {



        }

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));

        //  joint1Pos = joint1Enc.get() * 2 * Math.PI;
//    joint2Pos = joint2Enc.get() * 2 * Math.PI;

//     m_Joint2.set(joint2Speed);

    }

    public void setJoint1(double power) {

        if ((power > 0 && !ls_joint1PosLimit.get()) || (power < 0 && !ls_joint1NegLimit.get())) {

            joint1Speed = 0;

        } else if (power > 0.2) {

            joint1Speed = 0.2;

        } else if (power < -0.2) {

            joint1Speed = -0.2;

        } else {

            joint1Speed = power;

        }

        mg_Joint1.set(joint1Speed);

        SmartDashboard.putNumber("Joint 1 Angle", joint1CurrentPosition());

    }

    public void setJoint2(double power) {

        if (power > 0.2) {

            joint2Speed = 0.2;

        } else if (power < -0.2) {

            joint2Speed = -0.2;

        } else {

            joint2Speed = power;

        }

        m_Joint2.set(joint2Speed);
        SmartDashboard.putNumber("Joint 2 Angle", joint2CurrentPosition());

    }

    public double joint1CurrentPosition() {

        return -m_Joint1_1.getEncoder().getPosition() / 64.0 * 360.0;

    }

    public double joint2CurrentPosition() {

        return (Robot.Joint2Enc.getPulseWidthPosition() - 351.0) / 4096.0 * 360.0 + joint1CurrentPosition(); // 0 = 490, 

    }

    public boolean isJoint1AtPosition() {

        if (joint1CurrentPosition() + 5 > Math.toDegrees(joint1Angle) && joint1CurrentPosition() - 5 < Math.toDegrees(joint1Angle)) return true;
        else return false;

    }

    public boolean isJoint2AtPosition() {

        if (joint2CurrentPosition() + 5 > Math.toDegrees(joint2Angle) && joint2CurrentPosition() - 5 < Math.toDegrees(joint2Angle)) return true;
        else return false;

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

}
