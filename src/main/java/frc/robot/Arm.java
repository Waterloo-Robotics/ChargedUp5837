package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm {

    /* Arm Motor Controllers */
    CANSparkMax m_Joint1_1 = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_Joint1_2 = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m_Joint2 = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    MotorControllerGroup mg_Joint1 = new MotorControllerGroup(m_Joint1_1, m_Joint1_2);

    // Limit Switches for joint 1
    DigitalInput ls_joint1PosLimit = new DigitalInput(0);
    DigitalInput ls_joint1NegLimit = new DigitalInput(1);

    public enum ArmState {

        conePickup, cubePickup,
        coneScoreGround, cubeScoreGround,
        coneScoreLow, cubeScoreLow,
        coneScoreHigh, cubeScoreHigh,
        home, homeSequence

    }

    double joint1kP = 0;
    double joint1kI = 0;
    double joint1kD = 0;
    PIDController joint1PID = new PIDController(joint1kP, joint1kI, joint1kD);

    double joint2kP = 0;
    double joint2kI = 0;
    double joint2kD = 0;
    PIDController joint2PID = new PIDController(joint2kP, joint2kI, joint2kD);

    public Arm() {



    }

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

        System.out.println("Joint 1 Angle: " + Math.toDegrees(joint1Angle));

        System.out.println("Joint 2 Angle: " + Math.toDegrees(joint2Angle));

    }

    double x, y = 0;
    double joint1Pos, joint2Pos = 0;
    double joint1Speed, joint2Speed = 0;
    public void updateArm(ArmState armState) {

        switch (armState) {

            case conePickup:
                x = 30;
                y = 5;
                break;

            case coneScoreLow:
                x = 28;
                y = 38;
                break;

            case home:
                x = 0;
                y = 0;
                joint1Angle = 0;
                joint2Angle = 0;
                break;

        }
        if (armState != ArmState.home) setArmCoordinates(x, y);
//    joint1PID.setSetpoint(joint1Angle);
//    joint2PID.setSetpoint(joint2Angle);

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("joint1Angle", Math.toDegrees(joint1Angle));
        SmartDashboard.putNumber("joint2Angle", Math.toDegrees(joint2Angle));

        //  joint1Pos = joint1Enc.get() * 2 * Math.PI;
//    joint2Pos = joint2Enc.get() * 2 * Math.PI;

        joint1Speed = joint1PID.calculate(joint1Pos);
//    joint2Speed = joint2PID.calculate(joint2Pos);

        setJoint1(joint1Speed);
//     m_Joint2.set(joint2Speed);

    }

    public void setJoint1(double power) {

        if ((power > 0 && !ls_joint1PosLimit.get()) || (power < 0 && !ls_joint1NegLimit.get())) {

            joint1Speed = 0;

        } else {

            joint1Speed = power;

        }

        mg_Joint1.set(joint1Speed);

    }

}
