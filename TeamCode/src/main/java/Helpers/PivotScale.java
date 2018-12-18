package Helpers;

/**
 * Created by HHS-ROBOTICS on 11/2/2018.
 */

public class PivotScale extends CommonMath
{
    private static double scales[][]={
            {.27, .27, .25, .21,  .18, .15, .15, .18, .21, .25,  .27,  .27,  .3,  .3, .3},
            {0, .05, .08, .11, .15, .18, .21, .24, .27, .31,  .36, .41, .46, .5, .55}
    };

    private int range;


    public static double getPow(int input){

        double[] curScale=scales[0];

        int index=(int)(abs(input)*(curScale.length-1)/2700);

        index=RangePlus.constrain(index,0,curScale.length-1);

        double dScale=curScale[index];



        return dScale;
    }


}

