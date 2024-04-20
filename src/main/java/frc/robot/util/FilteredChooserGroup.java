package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class FilteredChooserGroup {
    
    private MultiPartStringFilterer mpsf;
    private String name;
    private String tabName;
    private ChangableChooser[] choosers;
    private Consumer<String> listener; // listener.accept to call

    public FilteredChooserGroup(String tabName, String name, int layers, String... strings) {
        this.name = name;
        this.tabName = tabName;
        mpsf = new MultiPartStringFilterer(layers, strings);
        choosers = new ChangableChooser[layers];

        populateChooser(0, mpsf.getStringsForLayer(0));
    }

    private void populateChooser(int layer, String... parts) {
        
        choosers[layer] = new ChangableChooser(tabName, "Layer " + layer);
        
        choosers[layer].setOptions(parts);

        choosers[layer].onChange(s -> {
            if((layer + 1) < mpsf.getLayerCount()) {
                // populate next layer chooser
                String[] previousParts = new String[layer+1];
                for(int i = 0; i <= layer; i++) {
                    previousParts[i] = choosers[i].get();
                }
                populateChooser(layer + 1, mpsf.getStringsForLayer(layer + 1, previousParts));
            }

            // report an overall change
            String[] allParts = new String[mpsf.getLayerCount()];
            for(int i = 0; i < layer; i++) {
                allParts[i] = choosers[i].get();
            }
            listener.accept(MultiPartStringFilterer.assembleParts(allParts));
        });

        if((layer + 1) < mpsf.getLayerCount() && choosers[layer + 1] == null) {
            // populate next layer chooser
            String[] previousParts = new String[layer+1];
            for(int i = 0; i <= layer; i++) {
                previousParts[i] = choosers[i].get();
            }
            populateChooser(layer + 1, mpsf.getStringsForLayer(layer + 1, previousParts));
        }
    }

    public void onChange(Consumer<String> listener) {
        ErrorMessages.requireNonNullParam(listener, "listener", "onChange");
        this.listener = listener;
    }

    public void periodic() {
        for(int i = 0; i < choosers.length; i++) {
            choosers[i].periodic();
        }
    }
}
