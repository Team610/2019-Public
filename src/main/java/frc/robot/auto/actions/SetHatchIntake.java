package frc.robot.auto.actions;

import frc.robot.subsystems.Manipulator;

/**
 * SetHatchIntake
 */
public class SetHatchIntake extends RunOnceAction {

    private boolean state;

    public SetHatchIntake(boolean state) {
        this.state = state;
    }
    
    @Override
    public void runOnce() {
        Manipulator.getInstance().setToggleState(state);
    }
    
}