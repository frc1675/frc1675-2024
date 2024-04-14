package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class FilteredChooserGroup {
    
    private MultiPartStringFilterer mpsf;
    private String name;
    private ShuffleboardTab tab;
    private SendableChooser<String>[] choosers;
    private Consumer<String> listener; // listener.accept to call

    public FilteredChooserGroup(ShuffleboardTab tab, String name, int layers, String... strings) {
        this.name = name;
        this.tab = tab;
        mpsf = new MultiPartStringFilterer(layers, strings);
        choosers = new SendableChooser[layers];

        populateChooser(0, mpsf.getStringsForLayer(0));
    }

    private void populateChooser(int layer, String... parts) {
        if(choosers[layer] != null) {
            choosers[layer].close();
        }
        choosers[layer] = new SendableChooser<String>();
        for(int i = 0; i < parts.length; i++) {
            choosers[layer].addOption(parts[i], parts[i]);
        }

        choosers[layer].onChange(s -> {
            if((layer + 1) < mpsf.getLayerCount()) {
                // populate next layer chooser
                String[] previousParts = new String[layer];
                for(int i = 0; i < layer; i++) {
                    previousParts[i] = choosers[i].getSelected();
                }
                populateChooser(layer + 1, mpsf.getStringsForLayer(layer + 1, previousParts));
            }

            // report an overall change
            String[] allParts = new String[mpsf.getLayerCount()];
            for(int i = 0; i < layer; i++) {
                allParts[i] = choosers[i].getSelected();
            }
            listener.accept(MultiPartStringFilterer.assembleParts(allParts));
        });

        tab.add(name + layer, choosers[layer]);

        if((layer + 1) < mpsf.getLayerCount() && choosers[layer + 1] == null) {
            // populate next layer chooser
            String[] previousParts = new String[layer];
            for(int i = 0; i < layer; i++) {
                previousParts[i] = choosers[i].getSelected();
            }
            populateChooser(layer + 1, mpsf.getStringsForLayer(layer + 1, previousParts));
        }
    }

    public void onChange(Consumer<String> listener) {
        ErrorMessages.requireNonNullParam(listener, "listener", "onChange");
        this.listener = listener;
    }
}
