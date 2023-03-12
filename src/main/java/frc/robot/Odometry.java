package frc.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Odometry {
    /* Motor Controllers */
    private static MotorControllerGroup rightDrive;
    private static MotorControllerGroup leftDrive;
    private static DifferentialDrive robotDrive;

    /* Drive Encoders */
    public static SensorCollection rightEncoder;
    public static SensorCollection leftEncoder;

    /* Phyiscal Parameters */
    private static double countsPerRev = 4096;
    private static double wheelDiameter = 6.0;
    private static double wheelCircumference = wheelDiameter * Math.PI;
    private static double gearRatio = 1.0;

    /* PID Constants */
    private static double P_general_fwrd = 0.02;
    private static double P_general_turn = 0.03;
    private static double P_differential = 0.01;

    private static PIDController genPID = new PIDController(P_general_fwrd, 0, 0);
    private static PIDController diffPID = new PIDController(P_general_turn, 0, 0);

    /* Speed Constants */
    private static double MAX_POWER = 0.3;
    private static double MIN_POWER_STRAIGHT = 0.1;
    private static double MIN_POWER_TURN = 0;

    public double genPower;
    public double diffPower;

    public double distanceTravelled;
    public double error;
    public double differentialError;

    public double rightTravelled;
    public double leftTravelled;

    public double rightTarget;
    public double leftTarget;
    public double forwardTarget;

    public boolean destinationReached;
    private int destinationReachedPersistenceCounter;
    private int destinationReachedPersistenceCounterLimit = 15;

    public Odometry (MotorControllerGroup leftMotorGroup,
                     MotorControllerGroup rightMotorGroup,
                     DifferentialDrive drivebase,
                     SensorCollection leftEnc,
                     SensorCollection rightEnc)
    {
        Odometry.leftDrive = leftMotorGroup;
        Odometry.rightDrive = rightMotorGroup;
        Odometry.robotDrive = drivebase;
        Odometry.leftEncoder = leftEnc;
        Odometry.rightEncoder =  rightEnc;
        Odometry.genPID.setTolerance(1);
        Odometry.diffPID.setTolerance(1);
        Odometry.diffPID.setSetpoint(0);
    }

    public double[] update() {
        double[] powers = new double[2];

        /* If we haven't reached our target perform the calcualtions, otherwise stop drivebase */
        if (!this.destinationReached) {
            this.rightTravelled = toInches(Odometry.rightEncoder.getQuadraturePosition());
            this.leftTravelled = toInches(-Odometry.leftEncoder.getQuadraturePosition());

            this.distanceTravelled = (this.rightTravelled + this.leftTravelled) / 2.0;
            this.differentialError = this.rightTravelled - this.leftTravelled;

            this.genPower = -clipPowerStraight(Odometry.genPID.calculate(distanceTravelled));
            this.diffPower = clipPowerStraight(Odometry.diffPID.calculate(differentialError));
        } else {
            this.genPower = 0;
            this.diffPower = 0;
        }

        updateMoveFinished();

        /* Check if target reached */
        if (moveFinished()) {
            this.genPower = 0;
            this.diffPower = 0;
        }

        powers[0] = this.genPower;
        powers[1] = this.diffPower;

        return powers;

        // Robot.r_robotDrive.arcadeDrive(this.genPower, this.diffPower);
    }

    private void updateMoveFinished() {
        if (Math.abs(Odometry.genPID.getPositionError()) < Odometry.genPID.getPositionTolerance()) {
            if (destinationReachedPersistenceCounter < destinationReachedPersistenceCounterLimit) {
                destinationReachedPersistenceCounter++;
            }
        } else {
            if (destinationReachedPersistenceCounter > 0) {
                destinationReachedPersistenceCounter--;
            }
        }

        if (destinationReachedPersistenceCounter >= destinationReachedPersistenceCounterLimit) {
            this.destinationReached = true;
        }
        
    }

    public boolean moveFinished() {
        return this.destinationReached;
    }

    public void forward(double inches) {

        /* Reset static variables */
        resetEncoders();
        this.rightTravelled = 0;
        this.leftTravelled = 0;
        this.distanceTravelled = 0;

        this.genPower = 0;
        this.diffPower = 0;

        this.destinationReached = false;
        destinationReachedPersistenceCounter = 0;

        genPID.setSetpoint(inches);
    }

    private static double toInches(double encoderCount) {
        return (encoderCount / Odometry.countsPerRev) * Odometry.gearRatio * Odometry.wheelCircumference;
    }

    public static void resetEncoders() {
        Odometry.rightEncoder.setQuadraturePosition(0, 0);
        Odometry.leftEncoder.setQuadraturePosition(0, 0);
    }

    private static double clipPowerStraight(double power) {
        if (Math.abs(power) < Odometry.MIN_POWER_STRAIGHT) 
        {
            return Odometry.MIN_POWER_STRAIGHT * Math.signum(power);
        } else if (Math.abs(power) > Odometry.MAX_POWER) 
        {
            return Odometry.MAX_POWER * Math.signum(power);
        } else {
            return power;
        }
    }

    





}
