package Helpers;

/**
 * Created by HHS-ROBOTICS on 11/2/2018.
 */

public class Scale extends CommonMath
{
    private static double scales[][]={
            {0, .05, .08, .1,  .15, .20, .20, .25, .25, .30,  .3,  .4,  .5,  .6, 1.0},
            {0, .05, .08, .11, .15, .18, .21, .24, .27, .31,  .36, .41, .46, .5, .55}
    };

    public static void printScale(int scale){
        int access=scale-1;
        double[] curScale=scales[access];
        System.out.println();
        for(int i=0;i<curScale.length;i++){
            System.out.print(curScale[i]+", ");
            if((i+1)%5==0&&i>2){
                System.out.println();
            }
        }
    }

    public static void printScale(double scale){
        int access=(int)scale-1;
        double[] curScale=scales[access];
        System.out.println();
        for(int i=0;i<curScale.length;i++){
            System.out.print(curScale[i]+", ");
            if((i+1)%5==0&&i>2){
                System.out.println();
            }
        }
    }

    public static double scaleInput(int scale, double input){

        int access=scale-1;
        double[] curScale=scales[access];

        int index=(int)(abs(input)*(curScale.length-1));

        index=RangePlus.constrain(index,0,curScale.length-1);

        double dScale=curScale[index];
        dScale*=(input>=0?1:-1);



        return dScale;
    }


}

