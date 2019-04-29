package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action {

    private Action curAction;
    private final ArrayList<Action> remainingActions;

    public SeriesAction(List<Action> actions) {
        remainingActions = new ArrayList<>(actions);
        curAction = null;
    }

    @Override
    public boolean isFinished() {
        return remainingActions.isEmpty() && curAction == null;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        if (curAction == null) {
            if (remainingActions.isEmpty()) {
                return;
            }

            curAction = remainingActions.remove(0);
            curAction.start();
        }

        curAction.update();

        if (curAction.isFinished()) {
            curAction.done();
            curAction = null;
        }
    }

    @Override
    public void done() {
    }
}
