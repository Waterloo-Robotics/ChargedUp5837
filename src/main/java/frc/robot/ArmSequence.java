package frc.robot;

public class ArmSequence {

  double[][] armStack;
  int buildIndex;
  int currentIndex;

  public ArmSequence() {

    this.armStack = new double[4][3];
    this.buildIndex = 0;
    this.currentIndex = 0;

  }

  public void addPosition(double x, double y, double z) {

    int index = this.buildIndex;
    armStack[index][0] = x;
    armStack[index][1] = y;
    armStack[index][2] = z;

    this.buildIndex++;

  }

  public void addPosition(ArmPosition position) {

    int index = this.buildIndex;
    armStack[index][0] = position.x;
    armStack[index][1] = position.y;
    armStack[index][2] = position.z;

    this.buildIndex++;

  }

  public ArmPosition nextPosition() {

    ArmPosition returnPosition = new ArmPosition(armStack[this.currentIndex][0], 
                                                armStack[this.currentIndex][1], 
                                                armStack[this.currentIndex][2]);

    if (this.currentIndex < this.buildIndex) {

        this.currentIndex++;

    }
    return returnPosition;

  }

  public boolean sequenceFinished() {

    return currentIndex == buildIndex;

  }

  public int getLength() {

    return buildIndex;

  }
    
}
