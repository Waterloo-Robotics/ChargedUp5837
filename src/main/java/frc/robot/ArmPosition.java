package frc.robot;

public class ArmPosition {
    double x;
    double y;
    double z;

    static double frontThreshold = 16;
    static double backThreshold = -16;

    public ArmPosition(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void setCoordinates(double x, double y, double z) {
        
        this.x = x;
        this.y = y;
        this.z = z;

    }

    public void incrementX(double step) {

        this.x += step;

    }

    public void incrementY(double step) {

        this.y += step;

    }

    public void incrementZ(double step) {

        this.z += step;

    }

    public double[] returnPosition()
    {
        double[] armPos = new double[3];
        armPos[0] = this.x;
        armPos[1] = this.y;
        armPos[2] = this.z;
        return armPos;
    }

    public int returnSide() {

        int side;
        if (this.x > frontThreshold) {
            
            side = 1;
        
        }
        else if (this.x < backThreshold) {
            
            side = -1;
        
        }
        else {
            
            side = 0;
        
        }

        return side;

    }

}
