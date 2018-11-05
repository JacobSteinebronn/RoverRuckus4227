package Helpers;

/**
 * Created by HHS-ROBOTICS on 11/2/2018.
 */

public class RangePlus extends CommonMath
{

    public static double constrain(double num, double low, double high){
        if(num<low){
            return low;
        }else if(num>high){
            return high;
        }else{
            return num;
        }
    }

    public static int constrain(int num, int low, int high){
        if(num<low){
            return low;
        }else if(num>high){
            return high;
        }else{
            return num;
        }
    }

    public static float constrain(float num, float low, float high){
        if(num<low){
            return low;
        }else if(num>high){
            return high;
        }else{
            return num;
        }
    }







    public static boolean within(double num, double low, double high){
        return constrain(num, low, high)==num;
    }

    public static boolean within(int num, int low, int high){
        return constrain(num, low, high)==num;
    }

    public static boolean within(float num, float low, float high){
        return constrain(num, low, high)==num;
    }



    public static double map(double num, double low1, double high1, double low2, double high2){
        double range1=high1-low1;
        double range2=high2-low2;
        double offset1=low1;
        double offset2=low2;
        double manipulate=num;
        if(constrain(num,low1,high1)==num){
            manipulate-=offset1;
            manipulate/=range1;
            manipulate*=range2;
            manipulate+=offset2;
            return manipulate;



        }else{
            return 0;
        }
    }

    public static int map(int num, int low1, int high1, int low2, int high2){
        int range1=high1-low1;
        int range2=high2-low2;
        int offset1=low1;
        int offset2=low2;
        int manipulate=num;
        if(constrain(num,low1,high1)==num){
            manipulate-=offset1;
            manipulate/=range1;
            manipulate*=range2;
            manipulate+=offset2;
            return manipulate;



        }else{
            return 0;
        }
    }

    public static float map(float num, float low1, float high1, float low2, float high2){
        float range1=high1-low1;
        float range2=high2-low2;
        float offset1=low1;
        float offset2=low2;
        float manipulate=num;
        if(constrain(num,low1,high1)==num){
            manipulate-=offset1;
            manipulate/=range1;
            manipulate*=range2;
            manipulate+=offset2;
            return manipulate;



        }else{
            return 0;
        }
    }



    public static double pushOut(double num, double low, double high){

        if(within(num,low,high)){

            double average=(low+high)/2;

            if(num<=average){
                return low;
            }else{
                return high;
            }


        }

        return num;
    }

    public static int pushOut(int num, int low, int high){

        if(within(num,low,high)){

            int average=(low+high)/2;

            if(num<=average){
                return low;
            }else{
                return high;
            }


        }

        return num;
    }

    public static float pushOut(float num, float low, float high){

        if(within(num,low,high)){

            float average=(low+high)/2;

            if(num<=average){
                return low;
            }else{
                return high;
            }


        }

        return num;
    }






}

