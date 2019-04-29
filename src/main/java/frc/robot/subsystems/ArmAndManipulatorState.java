package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Manipulator.ManipState;
import frc.util.Point;
import frc.util.Util;

public class ArmAndManipulatorState {
    public Point position;
    public ManipState state;
    public double manipAngle;

    public ArmAndManipulatorState(Point position, ManipState state, double manipAngle) {
        this.position = position;
        this.state = state;
        this.manipAngle = manipAngle;
    }

    public Point getPosition() {
        return position;
    }

    public ManipState getState() {
        return state;
    }

    public double getAngle() {
        return manipAngle;
    }

    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof ArmAndManipulatorState)) {
            return false;
        }
        ArmAndManipulatorState p = (ArmAndManipulatorState) obj;
        boolean equals = true;
        equals &= p.position.equals(getPosition());
        equals &= Util.withinEpsilon(getAngle(), p.getAngle(), Constants.Units.EPSILON);
        equals &= getState() == p.getState();
        return equals;
    }

    @Override
    public String toString() {
        return "Arm: " + position + " Manip State: " + state + " Manip Angle: " + manipAngle;
    }

}