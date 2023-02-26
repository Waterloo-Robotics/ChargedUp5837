package frc.robot;

public class ArmSequence {

  double[][] armStack;
  int buildIndex;
  int currentIndex;


  /* Constructor */
  public ArmSequence() {

    this.armStack = new double[4][3];
    this.buildIndex = 0;
    this.currentIndex = 0;

  }

  /* Add a position to the sequence */
  public void addPosition(double x, double y, double z) {

    /* Set x,y,z at the buildIndex */
    int index = this.buildIndex;
    armStack[index][0] = x;
    armStack[index][1] = y;
    armStack[index][2] = z;

    /* Increment buildIndex */
    this.buildIndex++;

  }

  /* Add a position to the sequence */
  public void addPosition(ArmPosition position) {
    
    /* Set x,y,z at the buildIndex */
    int index = this.buildIndex;
    armStack[index][0] = position.x;
    armStack[index][1] = position.y;
    armStack[index][2] = position.z;

    /* Increment buildIndex */
    this.buildIndex++;

  }

  /* Return next position in the sequence */
  public ArmPosition nextPosition() {

    /* Create a new ArmPostion object to store next position */
    ArmPosition returnPosition = new ArmPosition(this.armStack[this.currentIndex][0], 
                                                 this.armStack[this.currentIndex][1], 
                                                 this.armStack[this.currentIndex][2]);
    
    /* Only increment if currentIndex is less than sequence length
       this prevents a null position being returned */
    if (this.currentIndex < this.buildIndex) {

        this.currentIndex++;

    }

    /* Return position */
    return returnPosition;

  }

  /* Check if all positions have been requested */
  public boolean sequenceFinished() {

    return this.currentIndex == this.buildIndex;

  }

  /* Get the number of positions in the sequence */
  public int getLength() {

    return this.buildIndex;

  }
    
}
