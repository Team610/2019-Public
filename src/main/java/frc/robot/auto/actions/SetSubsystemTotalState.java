package frc.robot.auto.actions;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.SubsystemManager.TotalState;

/**
 * SetSubsystemManipState
 */
public class SetSubsystemTotalState extends RunOnceAction {

    private TotalState state;

    public SetSubsystemTotalState(TotalState state) {
        this.state = state;
    }

    @Override
    public void runOnce() {
        SubsystemManager.getInstance().setState(state);
    }

    
}