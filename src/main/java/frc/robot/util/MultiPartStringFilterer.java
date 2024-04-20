package frc.robot.util;

import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Actively filters a list of strings by "layers" - separate string "parts" delimited by dashes
 */
public class MultiPartStringFilterer {

    private int layers;
    private String[] fullStrings;

    public MultiPartStringFilterer(int layers, String[] strings) {
        this.layers = layers;
        this.fullStrings = strings;
    }

    public int getLayerCount() {
        return layers;
    }

    public String[] getStringsForLayer(int layer, String... previousStringParts) {
        Set<String> returnSet = new LinkedHashSet<String>();
        // first - if we dont have a number of previous strings equal to the requested layer, return an empty array.
        if (previousStringParts.length < layer) {
            return new String[0];
        }

        // assemble previousStrings into what prospective parts would start with
        String starter = assembleParts(previousStringParts);
        if (starter.length() > 0) starter += "-";

        for (int i = 0; i < fullStrings.length; i++) {
            if (fullStrings[i].startsWith(starter)) {
                returnSet.add(getNextPart(fullStrings[i], starter, layer));
            }
        }

        return returnSet.toArray(new String[0]);
    }

    public static String assembleParts(String... stringParts) {
        StringBuilder returnBuilder = new StringBuilder();
        for (int i = 0; i < stringParts.length; i++) {
            if (i != 0) {
                returnBuilder.append("-");
            }
            returnBuilder.append(stringParts[i]);
        }

        return returnBuilder.toString();
    }

    public String getNextPart(String fullString, String starter, int layersDeep) {
        String nextPart = "";

        if (layersDeep < layers && fullString.startsWith(starter)) {
            if (layers - layersDeep == 1) {
                // final layer, return rest of string
                nextPart = fullString.substring(starter.length());
            } else {
                // return from end of starter to next dash (or end of string)
                int nextPartEnd = fullString.indexOf("-", starter.length());
                nextPart =
                        fullString.substring(starter.length(), nextPartEnd != -1 ? nextPartEnd : fullString.length());
            }
        }

        return nextPart;
    }
}
