package org.firstinspires.ftc.teamcode.Ballistics;

import java.util.Scanner;
import org.firstinspires.ftc.teamcode.Ballistics.DragConstantCalculation;

public class main
{
    public static void main(String[] args)
    {
        double h, t;
        Scanner sc = new Scanner(System.in);
        System.out.print("Input height: ");
        h = sc.nextDouble();
        System.out.print("Input time: ");
        t = sc.nextDouble();
        double dragConst = DragConstantCalculation.dragConstant(t, h); // FIXME крч в кратце что то не так либо с
                                                                       // FIXME инициалиацией, либо с индексацией,
                                                                       // FIXME либо с юзом матриц
        System.out.printf("Drag constant is %f", dragConst);
    }
}
