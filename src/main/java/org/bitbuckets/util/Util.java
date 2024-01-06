package org.bitbuckets.util;

import xyz.auriium.mattlib2.log.ProcessPath;

import java.util.function.BiFunction;

/**
 * Math and physics identities we will never have to configure, so they dont go in mattlib2
 */
public class Util {

    public static final double MAX_VOLTAGE = 12.0;
    public static final BiFunction<String, Integer, String> RENAMER = (original,number) -> { //to make things look nice
        String[] asArray = ProcessPath.parse(original).asArray();
        String toAppend = asArray[1];
        asArray[1] = switch (number) {
            case 0 -> "fl";
            case 1 -> "fr";
            case 2 -> "bl";
            case 3 -> "br";
            default -> "h"+number;
        };
        return ProcessPath.of(asArray).append(toAppend).getAsTablePath();
    };


}
