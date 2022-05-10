package com.mmkrusniak.frame;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

/*
 * 1) Options are not often changed
 * 2) Options are independent
 * 3) Options are accessible concurrently
 * 4) Options are loosely categorized
 * 5) Options are absolutely global
 */

/**
 * A framework for static retrieval of option values. It's a bridge between the GUI and the backend, where all
 * parameters controlled by the user (and not by the backend) can be managed and rendered as Swing components.
 *
 * The options system is heavy, and it's intended for a specific type of values:
 *   - Values which are seldom changed, and usually by the user.
 *   - Values which do not depend on each other (although they may be hidden under some circumstances)
 *   - Values which are accessible concurrently.
 *   - Values which are loosely categorized.
 *   - Values which are absolutely global and should be able to be accessed anywhere.
 *
 *  Aside from storing values, the options framework also allows you to easily render a Swing panel for it and listen
 *  for changes. When an option is changed, all panels managing it graphically change as well.
 *
 *  Options are, generally speaking, typed dynamically: it's up to you to remember what type your options are.
 *
 *  Options are stored and retrieved statically. The lifecycle looks a bit like this:
 *   1.  {@link Option#register Register} an option. You give it a category, a name, a type, a display name, and some
 *   specs, in that order. (The option is thereafter referred to by its name and category.) The {@code specs} for the
 *   option depend on the type:
 *        - The first entry is always the default value, no matter the type.
 *        - In a numeric option, the specs are minimum value, maximum value, and step, respectively.
 *        - File, String, and Boolean options take no specs aside from the first.
 *        - With all other types, the specs specify acceptable values.
 *   2. {@link Option#listen Listen} to the option, specifying some actions to happen when it updates. You can also
 *   listen to an entire category, in which case the listener automatically listens to new entries in the category.
 *   If you allow the option to refresh, the listeners will also be called whenever the value is set but does not
 *   change (and a refresh button will appear alongside the rendered option).
 *   3. {@link Option#toPanel Render} the option to a Swing component, so it can be changed by the user. The
 *   component differs depending on the type specified for the option, but it always automatically changes the value
 *   of the option and updates all listeners of the object (and other generated panels) when doing so.
 *   4. {@link Option#get Get} the option on demand. You don't necessarily have to request it as the type which you
 *   specified it as first: for instance, you can register an option of type Object (so that it renders as a combo
 *   box), but actually retrieve it as a String (if the current value is, in fact, a String).
 *
 * The functionality is based on a similar parameter system from Python-based ROS.
 * @param <T> The type of a specific option (internal use only).
 */

@SuppressWarnings({"unused", "unchecked1"})
public class Option<T> {

    // One of the big problems we're trying to handle here is concurrency. We can't prevent changes in between
    // repeated uses of an option, but we *can* prevent inconsistencies from changes occurring at the same time.
    private static final ConcurrentHashMap<String, HashMap<String, Option<?>>> options = new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, List<ActionListener>> categoricalListeners =
            new ConcurrentHashMap<>();
    private static final ConcurrentHashMap<String, HashMap<Option<?>, Object>> categoricalDependencies =
            new ConcurrentHashMap<>();

    private final static boolean DEBUG_GUI = false;

    /**
     * Creates a new option in the system. It is henceforth accessed (globally) by the category and name given here.
     * @param category The category name for the new option. If no other options exist in the category, it is created.
     * @param name The name of the option. Ideally something short and clear.
     * @param type The type of the option.
     * @param displayName A human-readable name for the option.
     * @param specs Specifications for the option, which vary depending on its type:
     *          - The first entry is always the default value, no matter the type.
     *          - In a numeric option, the specs are minimum value, maximum value, and step, respectively.
     *          - File, Boolean, and String options take no specs aside from the first.
     *          - With all other types, the specs specify acceptable values.
     * @param <C> Type parameter for the option (ensuring that specs and type match).
     */
    @SafeVarargs
    public static <C> void register(String category, String name, Class<C> type, String displayName, C... specs) {
        options.computeIfAbsent(category, k -> new HashMap<>());
        categoricalListeners.computeIfAbsent(category, k -> new ArrayList<>());

        Option<C> newOption = new Option<>(type, name, displayName, specs);
        options.get(category).put(name, newOption);
        for(ActionListener al: categoricalListeners.get(category)) newOption.addListener(al);
        if(categoricalDependencies.get(category) != null)
                for(Option<?> p: categoricalDependencies.get(category).keySet()) newOption.depend(p,
                        categoricalDependencies.get(category).get(p));
    }

    @SafeVarargs
    public static <C> void registerList(String category, String name, Class<C[]> type, String displayName,
                                        String instanceDisplayName, C... specs) {
        options.computeIfAbsent(category, k -> new HashMap<>());
        categoricalListeners.computeIfAbsent(category, k -> new ArrayList<>());

        Option<C[]> newOption = new ListOption<>(type, name, displayName, 0, specs);
        options.get(category).put(name, newOption);
        for(ActionListener al: categoricalListeners.get(category)) newOption.addListener(al);
        if(categoricalDependencies.get(category) != null)
            for(Option<?> p: categoricalDependencies.get(category).keySet()) newOption.depend(p,
                    categoricalDependencies.get(category).get(p));
    }

    /**
     * Gives the current value of the option as some specified type.
     * @param category The category name for the option.
     * @param name The name of the option.
     * @param type The type to return the object as.
     * @param <C> Type parameter for the option (ensuring that specs and type match).
     * @throws ClassCastException If the current value of the option cannot be cast to the specified one.
     * @throws IllegalArgumentException If there is no object with the specified identifier.
     */
    public static <C> C get(String category, String name, Class<C> type) {
        checkExists(category, name);
        return type.cast(options.get(category).get(name).getValue());
    }

    /**
     * Sets the current value of the option. This also alerts the option's listeners to a change (and if the option
     * is allowed to be refreshed, the alert occurs even if the new value is equal to the existing one.)
     * @param category The category name for the option.
     * @param name The name of the option.
     * @param value The value to which to set the option.
     * @throws ClassCastException If the provided value cannot be cast to the type of the object.
     * @throws IllegalArgumentException If there is no object with the specified identifier.
     */
    public static void set(String category, String name, Object value) {
        checkExists(category, name);
        options.get(category).get(name).setValue(value);
    }

    /**
     * Renders the option into a JPanel. The rendering depends on the type specified for the option when it was
     * created:
     * @param category The category name for the option.
     * @param name The name of the option.
     * @throws IllegalArgumentException If there is no object with the specified identifier.
     */
    public static JPanel getPanel(String category, String name) {
        checkExists(category, name);
        return options.get(category).get(name).toPanel();
    }

    /**
     * Sets up a listener (or multiple) for an option. When the option is changed in the future (or when it is set to
     * its existing value, if refresh is allowed), the listener is called. Listeners are called in the order in which
     * they are added.
     * @param category The category name for the option.
     * @param name The name of the option.
     * @param listeners One or more listeners to add to the option.
     * @throws IllegalArgumentException If there is no object with the specified identifier.
     */
    public static void listen(String category, String name, ActionListener... listeners) {
        checkExists(category, name);
        for(ActionListener a: listeners) {
            options.get(category).get(name).addListener(a);
        }
    }

    /**
     * Sets up a listener (or multiple) for an entire category. The listener is immediately added to all options in
     * that category, and is added to all future options in that category as well.
     * @param category An option category name. There need not be any options in the category.
     * @param listeners One or more listeners.
     */
    public static void listenAll(String category, ActionListener... listeners) {
        options.computeIfAbsent(category, k -> new HashMap<>());
        for(ActionListener a: listeners) {
            // TBH, never heard of computeIfAbsent. Great IDE suggestion, thanks IntelliJ.
            categoricalListeners.computeIfAbsent(category,
                    k -> new ArrayList<>());
            for (Option<?> o : options.get(category).values()) o.addListener(a);
            categoricalListeners.get(category).add(a);
        }
    }

    public static void ignore(String category, String name, ActionListener... listeners) {
        checkExists(category, name);
        for(ActionListener a: listeners) {
            options.get(category).get(name).removeListener(a);
        }
    }

    public static void ignoreAll(String category, ActionListener... listeners) {
        options.computeIfAbsent(category, k -> new HashMap<>());
        for(ActionListener a: listeners) {
            for (Option<?> o : options.get(category).values()) o.removeListener(a);
            categoricalListeners.get(category).remove(a);
        }
    }

    /**
     * Gives whether an option with the specified identifier exists.
     * @param category The category name for the option.
     * @param name The name of the option.
     * @return Whether such an option exists.
     */
    public static boolean exists(String category, String name) {
        return options.get(category) != null && options.get(category).get(name) != null;
    }


    /**
     * Sets an option to alert listeners when it is set to its existing value. Also causes a refresh button to be
     * added when the option is rendered as a JPanel.
     * @param category The category name for the option.
     * @param name The name of the option.
     * @throws IllegalArgumentException If there is no object with the specified identifier.
     */
    public static void allowRefresh(String category, String name) {
        checkExists(category, name);
        options.get(category).get(name).allowRefresh();
    }

    public static void depend(String cat1, String name1, String cat2, String name2, Object value) {
        checkExists(cat1, name1);
        checkExists(cat2, name2);
        options.get(cat1).get(name1).depend(options.get(cat2).get(name2), value);
    }

    public static void dependAll(String cat, String optionCat, String optionName, Object value) {
        checkExists(optionCat, optionName);
        Option<?> target = options.get(optionCat).get(optionName);
        for(Option<?> p: options.get(cat).values()) p.depend(target, value);
        categoricalDependencies.computeIfAbsent(cat, k -> new HashMap<>());
        categoricalDependencies.get(cat).put(target, value);
    }

    private static void checkExists(String category, String name) {
        if (options.get(category) == null)
            throw new IllegalArgumentException("Options category \"" + category + "\" does not exist.");
        if(options.get(category).get(name) == null)
            throw new IllegalArgumentException("Option \"" + category + ":" + name + "\" does not exist.");
    }


    // I'm keeping these momentarily while I figure out how to do the new energy model

    @Deprecated public static double cruiseAltitude = 50;
    @Deprecated public static double cruiseSpeed = 12;
    @Deprecated public static double tiltAngle = 55.0;
    @Deprecated public static double acceleration = 1.0;
    @Deprecated public static double deceleration = 1.0;
    @Deprecated public static double gratitudeAcceleration = 9.8;
    @Deprecated public static double bankedTurningRadius =
            Math.pow(cruiseSpeed,2)/(gratitudeAcceleration *Math.tan(tiltAngle));
    @Deprecated public static double turningPower = 225;
    @Deprecated public static double angleSpeed = 2.1;


    // ----------------------------------------------------------------------------------------------------------------
    // In the back end of things it's all object oriented, of course. So while you should interact with Option
    // statically, there are actually individual capital-O Option objects that do the heavy lifting.
    // But the constructor's private for a reason! Maybe it's an antipattern, but you really shouldn't be
    // instantiating these.

    protected T value;
    protected boolean refresh;
    protected T[] specs;
    protected Class<T> type;
    protected String name;
    protected String displayName;
    protected final List<ActionListener> listeners = new ArrayList<>();
    protected final List<WeakReference<JComponent>> components = new ArrayList<>();
    protected final List<WeakReference<JPanel>> panels = new ArrayList<>();
    protected final HashMap<Option<?>, Object> dependencies = new HashMap<>();

    // Weak references (generics too) are IMHO usually a bad sign - I may be doing something really complicated to
    // accomplish something simple. But I also don't feel like managing all these options by hand and having to
    // remember where they all are, so I've traded a bunch of little code smells for a big one I guess?

    protected Option() {}

    @SafeVarargs
    protected Option(Class<T> type, String name, String displayName, T... specs) {
        this.type = type;
        if(specs != null && specs.length > 0) this.value = specs[0];
        else this.value = null;
        this.displayName = displayName;
        this.specs = specs;
        this.name = name;
        this.refresh = false;
    }

    protected synchronized JPanel toPanel() {
        final JPanel result = new JPanel();

        JComponent component = getComponentFor(type, this::setValue, this::getValue, specs);
        components.add(new WeakReference<>(component));
        result.add(component);

        if(refresh) {
            JButton refreshButton = (new JButton("⟳"));
            refreshButton.addActionListener((ActionEvent a) -> alertListeners());
            result.add(refreshButton);
            if(DEBUG_GUI) refreshButton.setBackground(Color.MAGENTA);
        }
        result.setPreferredSize(result.getMinimumSize());
        result.setVisible(areDependenciesMet());
        panels.add(new WeakReference<>(result));

        if(value != null && Displayable.class.isAssignableFrom(type) && ((Displayable) value).isBordered()) {
            result.setBorder(new TitledBorder(displayName));
        } else if(! displayName.equals("")) result.add(new JLabel(displayName + ":"), 0);

        return result;
    }

    protected synchronized void addListener(ActionListener l) {
        listeners.add(l);
    }
    protected synchronized void removeListener(ActionListener l) {
        listeners.remove(l);
        System.out.println("Listener removed");
    }

    @SuppressWarnings("unchecked")
    protected synchronized void setValue(Object value) {
        if(value != null && ! type.isAssignableFrom(value.getClass()))
            throw new IllegalArgumentException("Cannot assign value of type " + value.getClass().getName() +
                    " to option of type " + type.getName());
        this.value = (T) value;
        updateComponents();
        redrawPanels();
        alertListeners();
    }

    protected synchronized void depend(Option<?> option, Object value) {
        dependencies.computeIfAbsent(option, k -> new ArrayList<>());
        dependencies.put(option, value);
        option.addListener(e -> redrawPanels());
        redrawPanels();
    }

    protected synchronized boolean areDependenciesMet() {
        for(Option<?> p: dependencies.keySet()) {
            if(! p.value.equals(dependencies.get(p))) return false;
        }
        return true;
    }

    protected synchronized void alertListeners() {
        // Fun fact: The foreach loop has to be outside the thread - otherwise, if you ignore this option as part
        // of your response when listening, you'll concurrently modify the list of listeners!
        for(ActionListener al: listeners)
            new Thread(() -> {
            al.actionPerformed(new ActionEvent(this, ActionEvent.ACTION_PERFORMED, name));
            }, "option_alert_" + name).start();
    }

    protected synchronized T getValue() { return value; }
    protected synchronized void allowRefresh() { refresh = true;}

    @SuppressWarnings("unchecked")
    protected synchronized void redrawPanels() {
        List<WeakReference<JPanel>> removablePanels = new ArrayList<>();
        for(WeakReference<JPanel> r: panels) {
            JComponent c = r.get();
            if(c != null) {
                c.setVisible(areDependenciesMet());
                c.repaint();
            } else removablePanels.add(r);
        }
        panels.removeAll(removablePanels);
    }

    protected synchronized void updateComponents() {
        List<WeakReference<JComponent>> removable = new ArrayList<>();
        for(WeakReference<JComponent> r: components) {
            JComponent c = r.get();
            if(c != null) {
                if(Displayable.class.isAssignableFrom(type)) c.firePropertyChange(name, 0, 0);
                else if(Number.class.isAssignableFrom(type)) ((JSpinner) c).setValue(value);
                else if(String.class.isAssignableFrom(type)) ((JTextField) c).setText((String) value);
                else if(Boolean.class.isAssignableFrom(type)) ((JCheckBox) c).setSelected((Boolean) value);
                else ((JComboBox<T>) c).setSelectedItem(value);
            } else removable.add(r);
        }
        components.removeAll(removable);
    }

    protected static <Q> JComponent getComponentFor(Class<Q> type, Consumer<Object> setter, Supplier<Object> getter,
                                                    Q[] specs) {

        Q value = type.cast(getter.get());

        // If the class happens to be intrinsically displayable, absolutely use that! In this framework classes are
        // allowed to choose how they're displayed. (But some preexisting classes have implementations here too - for
        // instance, you can still display an Integer even though it's not a Displayable.)
        // Only works if the value is not null; that's an irritating edge case I can't really handle right now.
        if(Displayable.class.isAssignableFrom(type) && value != null) {
            JPanel panel =  ((Displayable) value).toPanel(setter, getter, specs);
            if(DEBUG_GUI) panel.setBackground(Color.CYAN);
            return panel;
        }
        else if(Double.class.isAssignableFrom(type)) {
            final JSpinner spinner = new JSpinner(new SpinnerNumberModel(
                    (double) value, (double) specs[1], (double) specs[2], (double) specs[3]));
            spinner.addChangeListener(changeEvent -> setter.accept(spinner.getValue()));
            Component mySpinnerEditor = spinner.getEditor();
            JFormattedTextField tf = ((JSpinner.DefaultEditor) mySpinnerEditor).getTextField();
            // JSpinner is too dumb to pick a reasonable number of columns apparently, so here's how we do it here:
            // Maximum length left of decimal point + length right of decimal point on change - 2 for the "0." + 1 for
            //   possible minus sign
            // I think this is the ugliest code I've ever written. Lol, casting something once to auto-unbox and again
            // to actually cast, that's horrible
            tf.setColumns((((double) specs[3])%1.0 + "").length() + (""+(Math.max((int) (double) specs[2],
                    (int) (double) specs[1]))).length() - 1);
            if(DEBUG_GUI) spinner.setBackground(Color.ORANGE);
            return spinner;
        }
        else if(Integer.class.isAssignableFrom(type)) {
            final JSpinner spinner = new JSpinner(new SpinnerNumberModel(
                    (int) value, (int) specs[1], (int) specs[2], (int) specs[3]));
            spinner.addChangeListener(changeEvent -> setter.accept(spinner.getValue()));
            if(DEBUG_GUI) spinner.setBackground(Color.RED);
            return spinner;
        }
        // JFileChooser for files
        else if(File.class.isAssignableFrom(type)) {
            final JButton button = new JButton("Browse...");
            button.addActionListener(e -> {
                final JFileChooser fc = new JFileChooser();
                if (fc.showOpenDialog(button) == JFileChooser.APPROVE_OPTION) {
                    setter.accept(fc.getSelectedFile());
                }
            });
            return button;
        }
        // JTextField for strings
        else if(String.class.isAssignableFrom(type)) {
            final JTextField text = new JTextField((String) value);
            text.addActionListener(e -> setter.accept(text.getText()));
            if(DEBUG_GUI) text.setBackground(Color.YELLOW);
            return text;
        }
        // JCheckbox for booleans
        else if(Boolean.class.isAssignableFrom(type)) {
            final JCheckBox box = new JCheckBox();
            box.setSelected((Boolean) value);
            box.addActionListener(e -> setter.accept(box.isSelected()));
            if(DEBUG_GUI) box.setBackground(Color.BLUE);
            return box;
        }
        // JColor for colors
        else if(Color.class.isAssignableFrom(type)) {
            final JColorChooser colorChooser = new JColorChooser((Color) value);
            colorChooser.getSelectionModel().addChangeListener(e -> setter.accept(colorChooser.getColor()));
            return colorChooser;
        }
        // JComboBox for anything else
        else {
            final JComboBox<Q> box = new JComboBox<>(specs);
            box.setSelectedItem(value);
            box.addActionListener(e -> setter.accept(box.getSelectedItem()));
            if(DEBUG_GUI) box.setBackground(Color.GREEN);
            return box;
        }
    }
}