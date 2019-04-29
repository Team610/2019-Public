package frc.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import frc.util.Toggle;

/**
 * ToggleTest
 */
public class ToggleTest {

    @Test
    public void toggleTest() {
        Toggle t = new Toggle();
        assertFalse(t.getToggleState());
        t.update(true);
        assertTrue(t.getToggleState());
        t.update(true);
        assertTrue(t.getToggleState());
        t.update(false);
        assertTrue(t.getToggleState());
        t.update(false);
        assertTrue(t.getToggleState());
        t.update(true);
        assertFalse(t.getToggleState());
        t.update(true);
        assertFalse(t.getToggleState());
    }

}