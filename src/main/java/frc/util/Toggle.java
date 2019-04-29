package frc.util;

public class Toggle {

    private boolean alreadyUpdated;
    private boolean toggleState;

    public Toggle() {
        this.alreadyUpdated = false;
        this.toggleState = false;
    }

    public void update(boolean condition) {
        if (condition && !alreadyUpdated) {
            alreadyUpdated = true;
            toggleState = !toggleState;
        }

        if (!condition) {
            alreadyUpdated = false;
        }
    }

    public void invertToggleState() {
        this.toggleState = !this.toggleState;
    }

    public void setToggleState(boolean state) {
        this.toggleState = state;
    }

    public boolean getToggleState() {
        return toggleState;
    }

}