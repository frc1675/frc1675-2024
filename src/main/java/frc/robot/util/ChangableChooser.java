package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.Arrays;
import java.util.function.Consumer;

/** HEAVILY inspired by 6328's SwitchableChooser */
public class ChangableChooser {

    private static final String placeholder = "<NA>";

    private String[] options = new String[] {placeholder};
    private String active = placeholder;
    private String previousActive = placeholder;

    private final StringPublisher namePublisher;
    private final StringPublisher typePublisher;
    private final StringArrayPublisher optionsPublisher;
    private final StringPublisher defaultPublisher;
    private final StringPublisher activePublisher;
    private final StringPublisher selectedPublisher;
    private StringSubscriber selectedInput;

    private Consumer<String> changeLambda;

    public ChangableChooser(String tab, String name) {
        var table = NetworkTableInstance.getDefault()
                .getTable("/Shuffleboard")
                .getSubTable(tab)
                .getSubTable(name);
        namePublisher = table.getStringTopic(".name").publish();
        typePublisher = table.getStringTopic(".type").publish();
        optionsPublisher = table.getStringArrayTopic("options").publish();
        defaultPublisher = table.getStringTopic("default").publish();
        activePublisher = table.getStringTopic("active").publish();
        selectedPublisher = table.getStringTopic("selected").publish();
        selectedInput = table.getStringTopic("selected").subscribe(this.options[0]);

        namePublisher.set(name);
        typePublisher.set("String Chooser");
        optionsPublisher.set(this.options);
        defaultPublisher.set(this.options[0]);
        activePublisher.set(this.options[0]);
        selectedPublisher.set(this.options[0]);
    }

    /** Updates the set of available options. */
    public void setOptions(String[] options) {
        if (Arrays.equals(options, this.options)) {
            return;
        }
        this.options = options.length == 0 ? new String[] {placeholder} : options;
        optionsPublisher.set(this.options);
        periodic();
    }

    /** Returns the selected option. */
    public String get() {
        return active == placeholder ? null : active;
    }

    public void periodic() {
        String selected = selectedInput.get();
        active = null;
        for (String option : options) {
            if (!option.equals(placeholder) && option.equals(selected)) {
                active = option;
            }
        }
        if (active == null) {
            active = options[0];
            selectedPublisher.set(active);
        }
        defaultPublisher.set(active);
        activePublisher.set(active);
        if (previousActive != active && changeLambda != null) {
            changeLambda.accept(active);
        }
        previousActive = active;
    }

    public void onChange(Consumer<String> changeLambda) {
        this.changeLambda = changeLambda;
    }
}
