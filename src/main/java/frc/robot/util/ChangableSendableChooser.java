package frc.robot.util;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * Basically a fork of SendableChooser that allows you to remove/clear the options.
 *
 * @param <V> The type of the values to be stored
 */
public class ChangableSendableChooser<V> implements Sendable, AutoCloseable {
    /** The key for the default value. */
    private static final String DEFAULT = "default";

    /** The key for the selected option. */
    private static final String SELECTED = "selected";

    /** The key for the active option. */
    private static final String ACTIVE = "active";

    /** The key for the option array. */
    private static final String OPTIONS = "options";

    /** The key for the instance number. */
    private static final String INSTANCE = ".instance";

    /** A map linking strings to the objects they represent. */
    private final Map<String, V> m_map = new LinkedHashMap<>();

    private String m_defaultChoice = "";
    private final int m_instance;
    private String m_previousVal;
    private Consumer<V> m_listener;
    private static final AtomicInteger s_instances = new AtomicInteger();

    /** Instantiates a {@link SendableChooser}. */
    @SuppressWarnings("this-escape")
    public ChangableSendableChooser() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChangableChooser", m_instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard} on the desktop, the
     * object will appear as the given name.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void addOption(String name, V object) {
        m_map.put(name, object);
    }

    /**
     * Removes the given name and its object from the list of options.
     * If it was the default choice, the default choice reverts to its default (empty string).
     * If it was the current selected option, the selected option changes to the default choice.
     * @param name the name of the option to remove
     */
    public void removeOption(String name) {
        if (m_map.containsKey(name)) {
            if (m_defaultChoice.equals(name)) {
                m_defaultChoice = "";
            }

            if (m_selected.equals(name)) {
                m_selected = m_defaultChoice;
            }

            m_map.remove(name);
        }
    }

    /**
     * Removes all options from the chooser.
     * After this call, the default choice will revert to empty string.
     * After this call, setting any value will trigger onChange, even if the value is the same as the value before clearing.
     */
    public void clearOptions() {
        m_defaultChoice = "";
        m_previousVal = "";
        m_selected = m_defaultChoice;

        m_map.clear();
    }

    /**
     * Replaces the current chooser options with the given options.
     * After this call, the default choice will revert to empty string.
     * @param options map of (names to options) to add
     */
    public void setOptions(Map<String, V> options) {
        requireNonNullParam(options, "options", "setOptions");

        clearOptions();
        m_map.putAll(options);
    }

    /**
     * Replaces the current chooser options with the given options, then sets a default option.
     * @param options map of (names to options) to add
     * @param defaultName the name of the default option
     * @param defaultObject the default option
     */
    public void setOptions(Map<String, V> options, String defaultName, V defaultObject) {
        requireNonNullParam(options, "options", "setOptions");
        requireNonNullParam(defaultName, "defaultName", "setOptions");
        requireNonNullParam(defaultObject, "defaultObject", "setOptions");

        setOptions(options);
        setDefaultOption(defaultName, defaultObject);
        if (m_selected.equals("")) {
            m_selected = defaultName;
        }
    }

    /**
     * Adds the given object to the list of options and marks it as the default. Functionally, this is
     * very close to {@link #addOption(String, Object)} except that it will use this as the default
     * option if none other is explicitly selected.
     *
     * @param name the name of the option
     * @param object the option
     */
    public void setDefaultOption(String name, V object) {
        requireNonNullParam(name, "name", "setDefaultOption");

        m_defaultChoice = name;
        addOption(name, object);
        m_selected = m_defaultChoice;
    }

    /**
     * Returns the selected option. If there is none selected, it will return the default. If there is
     * none selected and no default, then it will return {@code null}.
     *
     * @return the option selected
     */
    public V getSelected() {
        m_mutex.lock();
        try {
            if (m_selected != null) {
                return m_map.get(m_selected);
            } else {
                return m_map.get(m_defaultChoice);
            }
        } finally {
            m_mutex.unlock();
        }
    }

    /**
     * Bind a listener that's called when the selected value changes. Only one listener can be bound.
     * Calling this function will replace the previous listener.
     *
     * @param listener The function to call that accepts the new value
     */
    public void onChange(Consumer<V> listener) {
        requireNonNullParam(listener, "listener", "onChange");
        m_mutex.lock();
        m_listener = listener;
        m_mutex.unlock();
    }

    private String m_selected;
    private final ReentrantLock m_mutex = new ReentrantLock();

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstInteger(INSTANCE, m_instance);
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                () -> {
                    m_mutex.lock();
                    try {
                        if (m_selected != null) {
                            return m_selected;
                        } else {
                            return m_defaultChoice;
                        }
                    } finally {
                        m_mutex.unlock();
                    }
                },
                null);
        builder.addStringProperty(
                SELECTED,
                () -> {
                    // SELECTED maintains a getter to keep a proper state and fire onChange when items are removed
                    // if the final item is removed, this handles gracefully (gets empty string and no onChange call)
                    V choice;
                    Consumer<V> listener;
                    String setSelectedTo;
                    m_mutex.lock();
                    try {
                        if (m_selected != null) {
                            setSelectedTo = m_selected;
                        } else {
                            setSelectedTo = m_defaultChoice;
                        }
                        if (!setSelectedTo.equals(m_previousVal)
                                && m_listener != null
                                && m_map.containsKey(setSelectedTo)) {
                            choice = m_map.get(setSelectedTo);
                            listener = m_listener;
                        } else {
                            choice = null;
                            listener = null;
                        }
                        m_previousVal = setSelectedTo;
                    } finally {
                        m_mutex.unlock();
                    }
                    if (listener != null) {
                        listener.accept(choice);
                    }
                    return setSelectedTo;
                },
                val -> {
                    V choice;
                    Consumer<V> listener;
                    m_mutex.lock();
                    try {
                        m_selected = val;
                        // If dashboard loads with a selected that isn't there anymore, reset it
                        if (!m_map.containsKey(m_selected)) {
                            m_selected = m_defaultChoice;
                        }
                        if (!m_selected.equals(m_previousVal) && m_listener != null) {
                            choice = m_map.get(val);
                            listener = m_listener;
                        } else {
                            choice = null;
                            listener = null;
                        }
                        m_previousVal = val;
                    } finally {
                        m_mutex.unlock();
                    }
                    if (listener != null) {
                        listener.accept(choice);
                    }
                });
    }
}
