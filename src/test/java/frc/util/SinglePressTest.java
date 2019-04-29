package frc.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import frc.util.SinglePress;

/**
 * SinglePressTest
 */
public class SinglePressTest {

    @Test
    public void SinglePress() {
        SinglePress sp = new SinglePress();
        assertFalse(sp.getState());
        sp.update(true);
        assertTrue(sp.getState());
        sp.update(true);
        assertFalse(sp.getState());
        sp.update(false);
        assertFalse(sp.getState());
        sp.update(false);
        assertFalse(sp.getState());
        sp.update(true);
        assertTrue(sp.getState());
    }
    
}