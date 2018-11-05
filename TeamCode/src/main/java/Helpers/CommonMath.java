package Helpers;

/**
 * Created by HHS-ROBOTICS on 11/2/2018.
 */

public class CommonMath
{

    // public CommonMath(int value){
    //     this.value=value;
    //     numMaths++;
    // }

    public static double square(double num){
        return num*num;
    }

    public static int square(int num){
        return num*num;
    }

    public static float square(float num){
        return num*num;
    }

    public static double abs(double num){
        return num*(num>=0?1:-1);
    }

    public static String toFraction(double num){
        double dec=num%1;
        double cur=dec;
        for(int i=2;i<1000;i++){
            cur=dec;
            cur*=i;
            if(roundTo(cur%1,1000)%1==0){
                if(num-dec==0){
                    return (int)cur+"/"+i;
                }else{
                    return (num-num%1)+" and "+round(cur)+"/"+i;
                }
            }
        }
        return "Non-reachable fraction";
    }

    public static double roundTo(double num, double place){
        double temp=num;
        temp*=place;
        temp=round(temp);
        temp/=place;
        return temp;
    }

    public static double roundTo(double num, int place){
        double temp=num;
        temp*=place;
        temp=round(temp);
        temp/=place;
        return temp;
    }

    public static float roundTo(float num, float place){
        float temp=num;
        temp*=place;
        temp=round(temp);
        temp/=place;
        return temp;
    }

    public static float roundTo(float num, int place){
        float temp=num;
        temp*=place;
        temp=round(temp);
        temp/=place;
        return temp;
    }




    public static int round(double num){
        if(num%1>=.5){
            return ciel(num);
        }
        return floor(num);
    }

    public static int round(float num){
        if(num%1>=.5){
            return ciel(num);
        }
        return floor(num);
    }


    public static int ciel(double num){
        return (int)num+1;
    }

    public static int floor(double num){
        return(int)num;
    }

    public static int ciel(float num){
        return (int)num+1;
    }

    public static int floor(float num){
        return(int)num;
    }





}


