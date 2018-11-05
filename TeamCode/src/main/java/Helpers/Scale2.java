package Helpers;


public class Scale2 extends CommonMath {
    private static double scales[][]={
            {.09, .1, .12, .14, .16, .19, .22, .25, .29, .33, .38, .44, .51, .58, .66, .74, .82, .91, 1, 1}
    };


    public static double scaleInput( int scale, double val){

        double ret=1;
        double[] curScale=scales[scale-1];
        if(val<0){
            ret=-1;
            val*=-1;
        }
        if(val<.05)return 0;


        int idx=(int)(20*val-1);
        ret*=curScale[idx]+(val%.05*20)*(idx<19?(curScale[idx+1]-curScale[idx]):0);

        return ret;

    }
}
