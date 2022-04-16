package org.firstinspires.ftc.teamcode.usbserial;

import java.math.BigInteger;
import java.util.Arrays;

public class ArrayUtils {

    static byte[] concatArray(byte[] array1, byte[] array2) {
        byte[] result = Arrays.copyOf(array1, array1.length + array2.length);
        System.arraycopy(array2, 0, result, array1.length, array2.length);
        return result;
    }

    static byte[] truncateArray(byte[] array, int newLength) {
        if (array.length < newLength) {
            return array;
        } else {
            byte[] truncated = new byte[newLength];
            System.arraycopy(array, 0, truncated, 0, newLength);
            return truncated;
        }
    }

    static int getIntFromBytes(byte[] bytes){
        return new BigInteger(bytes).intValue();
    }

}
