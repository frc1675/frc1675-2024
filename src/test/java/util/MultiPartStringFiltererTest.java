package util;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import frc.robot.util.MultiPartStringFilterer;

public class MultiPartStringFiltererTest {
    
    @Test
    public void assembleParts() {
        String test1 = MultiPartStringFilterer.assembleParts("SubA", "CloseABC", "MidCD");
        Assertions.assertEquals("SubA-CloseABC-MidCD", test1);

        // assembling nothing returns blank string
        String test2 = MultiPartStringFilterer.assembleParts();
        Assertions.assertEquals("", test2);

        // 0-length parts are fine
        String test3 = MultiPartStringFilterer.assembleParts("SubA", "", "MidCD");
        Assertions.assertEquals("SubA--MidCD", test3);
    }

    @Test
    public void getNextPart_normalUsage() {
        String[] fullStrings = new String[0];
        
        MultiPartStringFilterer sut = new MultiPartStringFilterer(3, fullStrings);

        // on the first layer, return the first part.
        String test1 = sut.getNextPart("SubA-CloseABC-MidCD-FarA", "", 0);
        Assertions.assertEquals("SubA", test1);

        String test2 = sut.getNextPart("SubA-CloseABC-MidCD-FarA", "SubA-", 1);
        Assertions.assertEquals("CloseABC", test2);

        // on the final layer, return the rest of the string even if there are dashes.
        String test3 = sut.getNextPart("SubA-CloseABC-MidCD-FarA", "SubA-CloseABC-", 2);
        Assertions.assertEquals("MidCD-FarA", test3);

        // get a normal final layer
        String test4 = sut.getNextPart("SubA-CloseABC-MidCD", "SubA-CloseABC-", 2);
        Assertions.assertEquals("MidCD", test4);

        // get a final layer for something that was shorter than the max layers
        String test5 = sut.getNextPart("SubA-CloseABC", "SubA-", 1);
        Assertions.assertEquals("CloseABC", test5);
    }

    @Test
    public void getNextPart_exceptionalCases() {
        String[] fullStrings = new String[0];
        
        MultiPartStringFilterer sut = new MultiPartStringFilterer(3, fullStrings);

        // asking with layersDeep equal to or greater than the MPSF layers should return empty string
        String test1 = sut.getNextPart("SubA-CloseABC-MidCD-FarA", "SubA-CloseABC-MidCD", 3);
        Assertions.assertEquals("", test1);

        // asking with a starter that doesn't match should return empty string
        String test2 = sut.getNextPart("SubA-CloseABC-MidCD-FarA", "SubB", 1);
        Assertions.assertEquals("", test2);

        // don't freak out at 0-length parts
        String test3 = sut.getNextPart("SubA--MidCD-FarA", "SubA", 1);
        Assertions.assertEquals("", test3);

        // asking for a next part when there are no more parts, also not a big deal
        String test4 = sut.getNextPart("SubA-CloseABC", "SubA-CloseABC", 2);
        Assertions.assertEquals("", test4);
    }

    @Test
    public void getStringsForLayer() {
        String[] fullStrings = new String[8];
        fullStrings[0] = "SubA-CloseABC-MidCD";
        fullStrings[1] = "SubA-CloseABC-MidCD-FarAB";
        fullStrings[2] = "SubA-CloseABC-MidDC";
        fullStrings[3] = "SubB-CloseABC-MidCD";
        fullStrings[4] = "SubB-CloseB";
        fullStrings[5] = "SubB-CloseCBA-MidCD";
        fullStrings[6] = "SubC--MidCD";
        fullStrings[7] = "SubC--MidDC";
        
        MultiPartStringFilterer sut = new MultiPartStringFilterer(3, fullStrings);

        // layer 0 is all normal. Duplicates are not shown.
        String[] test1 = sut.getStringsForLayer(0);
        Assertions.assertArrayEquals(new String[]{"SubA", "SubB", "SubC"}, test1);

        // layer 1 for SubA is also normal.
        String[] test2 = sut.getStringsForLayer(1, "SubA");
        Assertions.assertArrayEquals(new String[]{"CloseABC"}, test2);

        String[] test3 = sut.getStringsForLayer(2, "SubA", "CloseABC");
        Assertions.assertArrayEquals(new String[]{"MidCD", "MidCD-FarAB", "MidDC"}, test3);

        String[] test4 = sut.getStringsForLayer(1, "SubB");
        Assertions.assertArrayEquals(new String[]{"CloseABC", "CloseB", "CloseCBA"}, test4);

        String[] test5 = sut.getStringsForLayer(2, "SubB", "CloseABC");
        Assertions.assertArrayEquals(new String[]{"MidCD"}, test5);

        String[] test6 = sut.getStringsForLayer(2, "SubB", "CloseB");
        Assertions.assertArrayEquals(new String[]{}, test6);

        String[] test7 = sut.getStringsForLayer(1, "SubC");
        Assertions.assertArrayEquals(new String[]{""}, test7);

        String[] test8 = sut.getStringsForLayer(2, "SubC", "");
        Assertions.assertArrayEquals(new String[]{"MidCD", "MidDC"}, test8);

    }
}
