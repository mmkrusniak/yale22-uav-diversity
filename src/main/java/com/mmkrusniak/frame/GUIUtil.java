package com.mmkrusniak.frame;

import javax.swing.*;

/**
 * A utility class which generates Swing components with useful properties. Setting up Swing components is usually
 * verbose and hard to read, so that lifting is done here.
 */
public class GUIUtil {

    // Wow, I hate Swing. It's been like seven years and I still don't really know how to use it.

    /**
     * Provides a JPanel which expands to fill all the space given to it. It's similar to BoxLayout glue, but glue
     * seems to misbehave with GTK. (This will also misbehave if you add one to a component that doesn't
     * have an upper bound on area, like the view of a JScrollPane.)
     * @return A JPanel which expands to fill all the space given to it.
     */
    public static JPanel fillerPanel() {
        JPanel result = new JPanel();
        result.setPreferredSize(result.getMaximumSize());
        return result;
    }

    /**
     * Gives a simple horizontal line with some space around it.
     * @return A horizontal line.
     */
    public static JSeparator horizontalLine() {
        JSeparator result = new JSeparator(JSeparator.HORIZONTAL);
        result.setBorder(BorderFactory.createEmptyBorder(20, 0, 20, 0));
        return result;
    }

    /**
     * Gives a centered text label.
     * @param text The text in the label.
     * @return That text centered in a component.
     */
    public static JComponent centeredLabel(String text) {
        JPanel panel = new JPanel();
        panel.add(new JLabel(text, SwingConstants.CENTER));
        panel.setSize(panel.getMinimumSize());
        return panel;
    }
}
