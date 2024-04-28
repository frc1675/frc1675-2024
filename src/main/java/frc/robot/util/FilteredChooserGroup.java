package frc.robot.util;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.LinkedHashMap;
import java.util.function.Consumer;

public class FilteredChooserGroup {

    private MultiPartStringFilterer mpsf;
    private String name;
    private ShuffleboardTab tab;
    private ChangableSendableChooser<String>[] choosers;
    private Consumer<String> listener; // listener.accept to call

    public FilteredChooserGroup(ShuffleboardTab tab, String name, int layers, String... strings) {
        this.name = name;
        this.tab = tab;
        mpsf = new MultiPartStringFilterer(layers, strings);
        choosers = new ChangableSendableChooser[layers];

        // Initialize all choosers
        initChoosers();

        populateChooser(0, mpsf.getStringsForLayer(0));
    }

    private void initChoosers() {
        for (int layerToInit = 0; layerToInit < mpsf.getLayerCount(); layerToInit++) {
            // need to make layer final for the loop so we can refer to it in change trigger
            final int layer = layerToInit;

            choosers[layer] = new ChangableSendableChooser<>();

            // set up change trigger
            choosers[layer].onChange(s -> {
                // if the next layer exists (max layer is layercount - 1)
                if ((layer + 1) < mpsf.getLayerCount()) {
                    // populate next layer chooser
                    // we need to construct the parts leading up to this chooser.
                    // if we are first layer (0), there will be 1 part, and so on
                    String[] previousParts = new String[layer + 1];
                    for (int i = 0; i <= layer; i++) {
                        previousParts[i] = choosers[i].getSelected();
                    }

                    // use those parts to get the choices for the next layer
                    String[] nextLayerOptions = mpsf.getStringsForLayer(layer + 1, previousParts);

                    populateChooser(layer + 1, nextLayerOptions);
                }

                // report an overall change
                String[] allParts = new String[mpsf.getLayerCount()];
                for (int i = 0; i < mpsf.getLayerCount(); i++) {
                    allParts[i] = choosers[i].getSelected();
                }
                listener.accept(MultiPartStringFilterer.assembleParts(allParts));
            });

            tab.add(name + " " + layer, choosers[layer]);
        }
    }

    private void populateChooser(int layer, String... options) {
        LinkedHashMap<String, String> optionMap = new LinkedHashMap<>();
        for (int i = 0; i < options.length; i++) {
            optionMap.put(options[i], options[i]);
        }

        // replace option set
        choosers[layer].setOptions(optionMap);

        if (options.length > 0) {
            choosers[layer].setDefaultOption(options[0], options[0]);
        }

        // if there is a chooser for the next layer
        /*if ((layer + 1) < mpsf.getLayerCount() && choosers[layer + 1] == null) {
            // populate next layer chooser
            String[] previousParts = new String[layer + 1];
            for (int i = 0; i <= layer; i++) {
                previousParts[i] = choosers[i].getSelected();
            }
            populateChooser(layer + 1, mpsf.getStringsForLayer(layer + 1, previousParts));
        }*/
    }

    public void onChange(Consumer<String> listener) {
        ErrorMessages.requireNonNullParam(listener, "listener", "onChange");
        this.listener = listener;
    }
}
