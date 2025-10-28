package org.firstinspires.ftc.teamcode.Ballistics;

import org.firstinspires.ftc.teamcode.Ballistics.Math.GeneralMath;
import org.firstinspires.ftc.teamcode.Ballistics.Math.Vector;
import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.Function;
import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.Integral;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.DifferentialEquation;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.Matrix;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.MatrixFunction;

import java.util.OptionalDouble;

import static org.firstinspires.ftc.teamcode.Ballistics.Constants.*;

public class DragConstantCalculation
{
    private static class Calc extends MatrixFunction
    {
        private double t_total, h;
        private Matrix v0;

        public Calc(double t, double h, Matrix v0)
        {
            t_total = t;
            this.h = h;
            this.v0 = v0;
        }

        @Override
        public Matrix execute(MatrixFunction func, Matrix... args) // args[0].x() = DragConstant
        {
            class BallisticEquation extends DifferentialEquation {
                @Override
                public Matrix CauchyFunction(double t, MatrixFunction func) {
                    Matrix retval = new Matrix(3);

                    final double DragBigConstant = -AirDensity * args[0].x() * CrossSectionalArea / (2 * BallMass);

                    retval.element(0, 0, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(t)).Size() * func.executeIndexed(0, 0, NullFunctionMatrix, new Matrix(t)) * DragBigConstant)); // FIXME
                    retval.element(0, 1, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(t)).Size() * func.executeIndexed(0, 1, NullFunctionMatrix, new Matrix(t)) * DragBigConstant)); // FIXME пишет, что не может достать элемент (0, 1) из результата вычислений
                    retval.element(0, 2, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(t)).Size() * func.executeIndexed(0, 2, NullFunctionMatrix, new Matrix(t)) * DragBigConstant - Constants.g));

                    return retval;
                }
            }


                    Integral integrate = new Integral();
                    BallisticEquation ballisticEq = new BallisticEquation();
                    Matrix retval = new Matrix(1);

                    // Step 1: Solve SODE

                    class FuncY extends Function {
                        @Override
                        public double execute(Function func, double... args) {
                            return ballisticEq.executeIndexed(0, 1, NullFunctionMatrix, new Matrix(args[0], Constants.Step), v0); // FIXME вот тут бро не может вытащить индекс
                        }
                    }

                    // Step 2: Integrate function
                    retval.element(0, 0, OptionalDouble.of(integrate.execute(new FuncY(), 0, t_total, Step))); // FIXME ну и вся функция идёт нахуй

                    // Step 4: Add free coefficients
                    final Matrix freeCoefficients = new Matrix(-h);
                    retval = retval.add(freeCoefficients);

                    // Return
                    return retval;
        }
    }

    public static double dragConstant(double t, double h)
    {
        Calc cal = new Calc(t, h, new Matrix(3));

        return GeneralMath.NewtonSOE(cal, Precision, 1000, new Matrix(0.47), 1, Step).x();
    }
}
