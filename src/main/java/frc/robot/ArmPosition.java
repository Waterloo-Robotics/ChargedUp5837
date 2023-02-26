package frc.robot;

public class ArmPosition {
    /* Coordinates defining arm position */
    double x;
    double y;
    double z;

    /* Thresholds for determining what side of the robot a position is on */
    static double frontThreshold = 16;
    static double backThreshold = -16;

    /* Constructor */
    public ArmPosition(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /* Update x,y,z */
    public void setCoordinates(double x, double y, double z) {
        
        this.x = x;
        this.y = y;
        this.z = z;

    }

    /* Increment X by step */
    public void incrementX(double step) {

        this.x += step;

    }

    /* Increment Y by step */
    public void incrementY(double step) {

        this.y += step;

    }
    
    /* Increment Z by step */
    public void incrementZ(double step) {

        this.z += step;

    }

    /* Return ArmPosition as an array of doubles */
    public double[] returnPosition()
    {
        double[] armPos = new double[3];
        armPos[0] = this.x;
        armPos[1] = this.y;
        armPos[2] = this.z;
        return armPos;
    }

    /* Return which side of the robot the position is on
       1 - Front
       0 - Within Frame Perimeter
      -1 - Back 
     */
    public int returnSide() {

        int side;

        /* If the x is past the front threshold, set side to front */
        if (this.x > frontThreshold) {
            
            side = 1;
        
        }
        /* If x is past the back threshold, set side to back */
        else if (this.x < backThreshold) {
            
            side = -1;
        
        }
        /* If x is not in front or back, set side to within frame perimeter */
        else {
            
            side = 0;
        
        }

        return side;

    }

}
