package com.mmkrusniak.frame;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.lang.ref.WeakReference;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import static com.mmkrusniak.frame.GUIUtil.fillerPanel;

// Definitely package private
class ListOption<E> extends Option<E[]> {

    private final List<Option<E>> list = new ArrayList<>();
    private final Class<E> innerType;
    private final E[] innerSpecs;

    @SafeVarargs @SuppressWarnings("unchecked")
    protected ListOption(Class<E[]> type, String name, String displayName, int initSize,
                         E... specs) {
        this.value = specs;
        this.name = name;
        this.type = type;
        this.displayName = displayName;
        this.innerType = (Class<E>) type.getComponentType();
        this.innerSpecs = specs;
        for (int i = 0; i < initSize; i++) { list.add(getNewEntry()); }
    }

    protected JPanel toPanel() {
        final JPanel result = new JPanel();
        result.setLayout(new BoxLayout(result, BoxLayout.PAGE_AXIS));


        AtomicInteger height = new AtomicInteger();
        for (Option<E> p : list) {
            JPanel subPanel = wrap(p, result);
            height.addAndGet(subPanel.getHeight());
            result.add(subPanel);
        }
        JButton add = new JButton("+");
        add.addActionListener(e -> {
            Option<E> duplicate = getNewEntry();
            System.out.println(duplicate);
            list.add(duplicate);
            JPanel subPanel = wrap(duplicate, result);
            height.addAndGet(subPanel.getHeight());
            result.add(subPanel, result.getComponents().length-2);
            result.revalidate();
            redrawPanels();
        });
        result.add(add);
        result.add(fillerPanel());
        result.setPreferredSize(new Dimension(result.getWidth(), height.get()));

        if(refresh) {
            JButton refreshButton = (new JButton("⟳"));
            refreshButton.addActionListener((ActionEvent a) -> alertListeners());
            result.add(refreshButton);
        }
        result.setPreferredSize(result.getMinimumSize());
        result.setAlignmentX(Component.CENTER_ALIGNMENT);
        result.setVisible(areDependenciesMet());
        result.setBorder(BorderFactory.createTitledBorder(displayName));
        panels.add(new WeakReference<>(result));
        return result;
    }

    private static int entryNum = 0;
    private Option<E> getNewEntry() {
        Option<E> p = new Option<>(innerType, name + entryNum, "", innerSpecs);
        entryNum++;
        p.addListener(e -> alertListeners());
        return p;
    }

    private JPanel wrap(Option<E> p, JPanel parent) {
        JPanel wrapper = new JPanel();
        wrapper.setLayout(new BoxLayout(wrapper, BoxLayout.PAGE_AXIS));
        JComponent subPanel = p.toPanel();
        subPanel.setBorder(null);
        wrapper.add(subPanel);
        JButton remove = new JButton("–");
        subPanel.add(remove);
        wrapper.setBackground(Color.CYAN);
        wrapper.setAlignmentX(Component.CENTER_ALIGNMENT);
        wrapper.setPreferredSize(new Dimension(wrapper.getWidth(), subPanel.getMinimumSize().height));
        remove.addActionListener(e -> {
            parent.remove(wrapper);
            list.remove(p);
            alertListeners();
            parent.revalidate();
            redrawPanels();});
        return wrapper;
    }

    @Override
    protected synchronized void updateComponents() {
        for(Option<E> p: list) p.updateComponents();
    }

    @Override
    protected E[] getValue() {
        @SuppressWarnings("unchecked")
        final E[] result = (E[]) Array.newInstance(innerType, list.size());
        for (int i = 0; i < list.size(); i++) {
            result[i] = list.get(i).getValue();
        }
        return result;
    }

}
