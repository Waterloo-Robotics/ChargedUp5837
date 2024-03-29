package frc.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Odometry {
    /* Motor Controllers */

    /* Drive Encoders */
    public static SensorCollection rightEncoder;
    public static SensorCollection leftEncoder;

    /* Phyiscal Parameters */
    private static double countsPerRev = 4096;
    private static double wheelDiameter = 6.0;
    private static double wheelCircumference = wheelDiameter * Math.PI;
    private static double gearRatio = 1.0;

    /* PID Constants */
    private static double P_general_fwrd = 0.015;
    private static double P_general_turn = 0.019;
    private static double P_differential = 0.00000000000001;

    static PIDController genPID = new PIDController(P_general_fwrd, 0, 0);
    static PIDController diffPID = new PIDController(P_differential, 0, 0);

    /* Speed Constants */
    public static double MAX_POWER = 0.5;
    private static double MIN_POWER_STRAIGHT = 0.1;
    private static double MIN_POWER_TURN = 0.15;

    public double genPower;
    public double diffPower;

    public double rightPower;
    public double leftPower;

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
    private int destinationReachedPersistenceCounterLimit = 25;

    private boolean isStraight = true;

    double trackWidth = 28.0;
    double fullRotation = trackWidth * Math.PI; // the amount the wheels travel in a full rotation
    double countsPerRobotRev = fullRotation * (4096 / wheelCircumference);
    double countsPerDegree = countsPerRobotRev / 360;
    double inchesPerDegree = toInches(countsPerDegree);

    public Odometry (MotorControllerGroup leftMotorGroup,
                     MotorControllerGroup rightMotorGroup,
                     DifferentialDrive drivebase,
                     SensorCollection leftEnc,
                     SensorCollection rightEnc) {

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
            if (isStraight) this.leftTravelled = toInches(-Odometry.leftEncoder.getQuadraturePosition());
            else this.leftTravelled = toInches(Odometry.leftEncoder.getQuadraturePosition());

            this.distanceTravelled = (this.rightTravelled + this.leftTravelled) / 2.0;
            this.differentialError = this.rightTravelled - this.leftTravelled;

            if (isStraight) {

                this.genPower = -clipPowerStraight(Odometry.genPID.calculate(distanceTravelled));

            } else {

                this.genPower = -clipPowerTurn(Odometry.genPID.calculate(distanceTravelled));

            }
            this.diffPower = clipPowerStraight(Odometry.diffPID.calculate(differentialError));

            if (isStraight) {            
            
                this.rightPower = this.genPower - this.diffPower / 2.0;
                this.leftPower = this.genPower + this.diffPower / 2.0;
            
            } else {

                this.rightPower = this.genPower;
                this.leftPower = this.genPower;

            }

            /* Turn left */
            // if (this.diffPower > 0) {
            //     this.leftPower = this.genPower - clip(this.diffPower, this.genPower);
            //     this.rightPower = this.genPower;
            // } 
            // /* Turn right */
            // else {
            //     this.rightPower = this.genPower + clip(this.diffPower, this.genPower);
            //     this.leftPower = this.genPower;
            // }

        } else {

            this.genPower = 0;
            this.diffPower = 0;

            this.rightPower = 0;
            this.leftPower = 0;

        }

        updateMoveFinished();

        /* Check if target reached */
        if (moveFinished()) {

            this.genPower = 0;
            this.diffPower = 0;
            this.rightPower = 0;
            this.leftPower = 0;
        }

        powers[0] = this.rightPower;
        if (isStraight) powers[1] = this.leftPower;
        else powers[1] = -this.leftPower;

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

    public void straight(double inches) {

        genPID.setP(P_general_fwrd);
        this.isStraight = true;
        this.move(inches);

    }

    public void turn(double degrees) {

        genPID.setP(P_general_turn);
        this.isStraight = false;
        this.move(degrees * inchesPerDegree);

    }

    public void move(double inches) {

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

    public double[] balance(double currentAngle, double currentAccel) {

        double[] powers = new double[2];

        // if charge station starts changing balance (robot detects change in angle) stop movement
        // if (Math.abs(currentAccel) > 3.0) {

            // powers[0] = 0;
            // powers[1] = 0;

        // } else {

            // make sure charge station isn't balanced before applying power;
            if (Math.abs(currentAngle) > 2) {

                // insert code to drive and stuff
                powers[0] = 0.8 * Math.signum(currentAngle);
                powers[1] = 0.8 * Math.signum(currentAngle);

            } else {

                powers[0] = 0;
                powers[1] = 0;

            }

        // }

        return powers;

    }

    private static double toInches(double encoderCount) {

        return (encoderCount / Odometry.countsPerRev) * Odometry.gearRatio * Odometry.wheelCircumference;

    }

    public static void resetEncoders() {

        Odometry.rightEncoder.setQuadraturePosition(0, 0);
        Odometry.leftEncoder.setQuadraturePosition(0, 0);

    }

    private static double clipPowerStraight(double power) {

        if (Math.abs(power) < Odometry.MIN_POWER_STRAIGHT) {

            return Odometry.MIN_POWER_STRAIGHT * Math.signum(power);

        } else if (Math.abs(power) > Odometry.MAX_POWER) {

            return Odometry.MAX_POWER * Math.signum(power);

        } else {

            return power;

        }

    }

    private static double clipPowerTurn(double power) {

        if (Math.abs(power) < Odometry.MIN_POWER_TURN) {

            return Odometry.MIN_POWER_TURN * Math.signum(power);

        } else if (Math.abs(power) > Odometry.MAX_POWER) {

            return Odometry.MAX_POWER * Math.signum(power);

        } else {

            return power;

        }

    }

    private static double clip(double power, double clip) {
        if (Math.abs(power) > clip) {
            return clip * Math.signum(power);
        } else {
            return power;
        }
    }

    





}
