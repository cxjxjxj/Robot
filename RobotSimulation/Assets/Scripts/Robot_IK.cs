using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
///机器人求逆过程
///需要输入此位姿与上一个位姿
///调用时直接使用All函数
/// </summary>
public class Robot_IK : MonoBehaviour
{
    /// <summary>
    /// 总的一个操作，相当于main函数，输入xyzwpr得到机器人的6个关节角度
    /// </summary>
    public static double[] ALL(double[] IKInput, double[] oldtheta)
    {
        double[] PValue = { 670, 312, 1075, 225, 1280 };

        #region//存放了6个旋转矩阵
        List<Matrix> AllTMatrix = new List<Matrix>();
        Matrix T01 = new Matrix(4, 4, "T01"); AllTMatrix.Add(T01);
        Matrix T12 = new Matrix(4, 4, "T12"); AllTMatrix.Add(T12);
        Matrix T23 = new Matrix(4, 4, "T23"); AllTMatrix.Add(T23);
        Matrix T34 = new Matrix(4, 4, "T34"); AllTMatrix.Add(T34);
        Matrix T45 = new Matrix(4, 4, "T45"); AllTMatrix.Add(T45);
        Matrix T56 = new Matrix(4, 4, "T56"); AllTMatrix.Add(T56);
        #endregion

        //《1》求正解
        double[] theat = { 71.2, 34, -38.1, 33, -51.9, -161.2 };
        for (int i = 0; i < theat.Length; i++)
        {

            theat[i] = theat[i] / 180D * Math.PI;

        }
        Matrix T06FK = FK(theat, PValue, ref AllTMatrix);

        //《2》求反解
        //double[] IKInput = { 1534.9, 864.4, 2740.5, -153.8, -13.1, 166.8 };//0
        //double[] oldtheta = { 0, 0, 0, 0, 0, 0 };
        double[] Theta = IK(PValue, IKInput, oldtheta);

        return Theta;
    }

    /// <summary>
    /// 机器人逆解
    /// </summary>
    /// <param name="PValue">DH链表中机器人的尺寸</param>
    /// <param name="IKInput">位姿输入：xyzwpr</param>
    /// <param name="oldtheta">上一个转角</param>
    /// <returns>解的集合</returns>
    private static double[] IK(double[] PValue, double[] IKInput, double[] oldtheta)
    {
        //0、装解的集合    
        double[,] Theta = new double[8, 6];
        double[] TheatBest = new double[6];

        //1、初始化获得T
        Matrix T06 = DealOrinData(IKInput);

        #region//数据精度整理
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (Math.Abs(T06.Detail[i, j]) < 0.0001)
                {
                    T06.Detail[i, j] = 0;
                }
            }
        }
        #endregion

        //2、px、py、pz
        double PX = T06.Detail[0, 3];
        double PY = T06.Detail[1, 3];
        double PZ = T06.Detail[2, 3];

        //3、求theta1
        CalculateTheta1(T06, ref Theta);

        //4、求theta3
        for (int i = 0; i < Theta.GetLength(0); i++)
        {
            CalculateTheta3(Theta[i, 0], PX, PY, PZ, PValue, i, ref Theta);
        }

        //5、求theta2
        for (int i = 0; i < Theta.GetLength(0); i++)
        {
            CalculateTheta2(Theta[i, 0], Theta[i, 2], PX, PY, PZ, PValue, i, ref Theta);
        }

        //6、求theta5
        for (int i = 0; i < Theta.GetLength(0); i++)
        {
            CalculateTheta5(Theta[i, 0], Theta[i, 1], Theta[i, 2], PValue, T06, i, ref Theta);
        }

        //7、theta4、6的计算
        for (int i = 0; i < Theta.GetLength(0); i++)
        {
            CalculateTheta4_6(Theta[i, 0], Theta[i, 1], Theta[i, 2], Theta[i, 4], PValue, T06, i, oldtheta, ref Theta);
        }

        #region//Theta的打印
        for (int i = 0; i < Theta.GetLength(0); i++)
        {
            Debug.Log("第" + i + "个解：" + Theta[i, 0] + "  " + Theta[i, 1] + "  " + Theta[i, 2] + "  " + Theta[i, 3] + "  " + Theta[i, 4] + "  " + Theta[i, 5]);
        }
        #endregion

        //8、找最优的一组解
        TheatBest = BestResult(Theta, oldtheta);

        for (int i = 0; i < TheatBest.Length; i++)
        {
            Debug.Log("关节" + i + "的角度：" + TheatBest[i]);
        }

        return TheatBest;
    }

    /// <summary>
    /// 机器人正解
    /// </summary>
    /// <param name="theta">输入的六个关节角度</param>
    /// <param name="staticValue">DH链表中机器人的尺寸</param>
    /// <param name="TempMatrix">6个旋转矩阵的初始化</param>
    /// <returns>正解得出的T06</returns>
    private static Matrix FK(double[] theta, double[] staticValue, ref List<Matrix> TempMatrix)
    {
        Debug.Log("============================计算正解============================");
        Matrix ReturnMatrix = new Matrix(4, 4, "正解T06");
        #region//给6个位置矩阵赋值
        for (int i = 0; i < TempMatrix.Count; i++)
        {
            switch (i)
            {
                case 0:
                    TempMatrix[0].Detail[0, 0] = Math.Cos(theta[0]);
                    TempMatrix[0].Detail[0, 1] = -Math.Sin(theta[0]);
                    TempMatrix[0].Detail[0, 2] = 0;
                    TempMatrix[0].Detail[0, 3] = 0;
                    TempMatrix[0].Detail[1, 0] = Math.Sin(theta[0]);
                    TempMatrix[0].Detail[1, 1] = Math.Cos(theta[0]);
                    TempMatrix[0].Detail[1, 2] = 0;
                    TempMatrix[0].Detail[1, 3] = 0;
                    TempMatrix[0].Detail[2, 0] = 0;
                    TempMatrix[0].Detail[2, 1] = 0;
                    TempMatrix[0].Detail[2, 2] = 1;
                    TempMatrix[0].Detail[2, 3] = staticValue[0];
                    TempMatrix[0].Detail[3, 0] = 0;
                    TempMatrix[0].Detail[3, 1] = 0;
                    TempMatrix[0].Detail[3, 2] = 0;
                    TempMatrix[0].Detail[3, 3] = 1;
                    break;
                case 1:
                    TempMatrix[1].Detail[0, 0] = Math.Sin(theta[1]);
                    TempMatrix[1].Detail[0, 1] = Math.Cos(theta[1]);
                    TempMatrix[1].Detail[0, 2] = 0;
                    TempMatrix[1].Detail[0, 3] = staticValue[1];
                    TempMatrix[1].Detail[1, 0] = 0;
                    TempMatrix[1].Detail[1, 1] = 0;
                    TempMatrix[1].Detail[1, 2] = 1;
                    TempMatrix[1].Detail[1, 3] = 0;
                    TempMatrix[1].Detail[2, 0] = Math.Cos(theta[1]);
                    TempMatrix[1].Detail[2, 1] = -Math.Sin(theta[1]);
                    TempMatrix[1].Detail[2, 2] = 0;
                    TempMatrix[1].Detail[2, 3] = 0;
                    TempMatrix[1].Detail[3, 0] = 0;
                    TempMatrix[1].Detail[3, 1] = 0;
                    TempMatrix[1].Detail[3, 2] = 0;
                    TempMatrix[1].Detail[3, 3] = 1;
                    break;
                case 2:

                    TempMatrix[2].Detail[0, 0] = Math.Cos(theta[2] + theta[1]);
                    TempMatrix[2].Detail[0, 1] = -Math.Sin(theta[2] + theta[1]);
                    TempMatrix[2].Detail[0, 2] = 0;
                    TempMatrix[2].Detail[0, 3] = staticValue[2];
                    TempMatrix[2].Detail[1, 0] = -Math.Sin(theta[2] + theta[1]);
                    TempMatrix[2].Detail[1, 1] = -Math.Cos(theta[2] + theta[1]);
                    TempMatrix[2].Detail[1, 2] = 0;
                    TempMatrix[2].Detail[1, 3] = 0;
                    TempMatrix[2].Detail[2, 0] = 0;
                    TempMatrix[2].Detail[2, 1] = 0;
                    TempMatrix[2].Detail[2, 2] = -1;
                    TempMatrix[2].Detail[2, 3] = 0;
                    TempMatrix[2].Detail[3, 0] = 0;
                    TempMatrix[2].Detail[3, 1] = 0;
                    TempMatrix[2].Detail[3, 2] = 0;
                    TempMatrix[2].Detail[3, 3] = 1;
                    break;
                case 3:
                    TempMatrix[3].Detail[0, 0] = Math.Cos(theta[3]);
                    TempMatrix[3].Detail[0, 1] = -Math.Sin(theta[3]);
                    TempMatrix[3].Detail[0, 2] = 0;
                    TempMatrix[3].Detail[0, 3] = staticValue[3];
                    TempMatrix[3].Detail[1, 0] = 0;
                    TempMatrix[3].Detail[1, 1] = 0;
                    TempMatrix[3].Detail[1, 2] = 1;
                    TempMatrix[3].Detail[1, 3] = -staticValue[4];
                    TempMatrix[3].Detail[2, 0] = -Math.Sin(theta[3]); ;
                    TempMatrix[3].Detail[2, 1] = -Math.Cos(theta[3]);
                    TempMatrix[3].Detail[2, 2] = 0;
                    TempMatrix[3].Detail[2, 3] = 0;
                    TempMatrix[3].Detail[3, 0] = 0;
                    TempMatrix[3].Detail[3, 1] = 0;
                    TempMatrix[3].Detail[3, 2] = 0;
                    TempMatrix[3].Detail[3, 3] = 1;
                    break;
                case 4:
                    TempMatrix[4].Detail[0, 0] = Math.Cos(theta[4]);
                    TempMatrix[4].Detail[0, 1] = -Math.Sin(theta[4]);
                    TempMatrix[4].Detail[0, 2] = 0;
                    TempMatrix[4].Detail[0, 3] = 0;
                    TempMatrix[4].Detail[1, 0] = 0;
                    TempMatrix[4].Detail[1, 1] = 0;
                    TempMatrix[4].Detail[1, 2] = -1;
                    TempMatrix[4].Detail[1, 3] = 0;
                    TempMatrix[4].Detail[2, 0] = Math.Sin(theta[4]); ;
                    TempMatrix[4].Detail[2, 1] = Math.Cos(theta[4]);
                    TempMatrix[4].Detail[2, 2] = 0;
                    TempMatrix[4].Detail[2, 3] = 0;
                    TempMatrix[4].Detail[3, 0] = 0;
                    TempMatrix[4].Detail[3, 1] = 0;
                    TempMatrix[4].Detail[3, 2] = 0;
                    TempMatrix[4].Detail[3, 3] = 1;
                    break;
                case 5:
                    TempMatrix[5].Detail[0, 0] = Math.Cos(theta[5]);
                    TempMatrix[5].Detail[0, 1] = -Math.Sin(theta[5]);
                    TempMatrix[5].Detail[0, 2] = 0;
                    TempMatrix[5].Detail[0, 3] = 0;
                    TempMatrix[5].Detail[1, 0] = 0;
                    TempMatrix[5].Detail[1, 1] = 0;
                    TempMatrix[5].Detail[1, 2] = 1;
                    TempMatrix[5].Detail[1, 3] = 0;
                    TempMatrix[5].Detail[2, 0] = -Math.Sin(theta[5]); ;
                    TempMatrix[5].Detail[2, 1] = -Math.Cos(theta[5]);
                    TempMatrix[5].Detail[2, 2] = 0;
                    TempMatrix[5].Detail[2, 3] = 0;
                    TempMatrix[5].Detail[3, 0] = 0;
                    TempMatrix[5].Detail[3, 1] = 0;
                    TempMatrix[5].Detail[3, 2] = 0;
                    TempMatrix[5].Detail[3, 3] = 1;
                    break;
                default:
                    break;
            }
        }
        #endregion
        #region//打印6个赋值了的位姿矩阵，看是否正确
        //for (int j = 0; j < TempMatrix.Count; j++)
        //{
        //    Matrix item = TempMatrix[j];
        //    Debug.Log("这是矩阵：" + item.Name);
        //    for (int i = 0; i < item.getM; i++)
        //    {
        //        Debug.Log("第" + i + "行：" + item.Detail[i, 0] + "  " + item.Detail[i, 1] + "  " + item.Detail[i, 2]);
        //    }
        //}
        #endregion

        #region//6个位置矩阵相乘

        ReturnMatrix = TempMatrix[0];
        for (int i = 0; i < TempMatrix.Count - 1; i++)
        {

            ReturnMatrix = MatrixOperator.MatrixMulti(ReturnMatrix, TempMatrix[i + 1]);
        }
        #endregion

        for (int i = 0; i < ReturnMatrix.getM; i++)
        {
            Debug.Log(ReturnMatrix.Name + "的第" + i + "行：" + ReturnMatrix.Detail[i, 0] + "  " +
                ReturnMatrix.Detail[i, 1] + "  " + ReturnMatrix.Detail[i, 2] + "  " + ReturnMatrix.Detail[i, 3]);
        }

        Debug.Log("============================结束正解============================");
        return ReturnMatrix;

    }

    /// <summary>
    /// 输入数据xyzwpr换算为T06的计算
    /// </summary>
    /// <param name="IKInput"></param>
    /// <returns></returns>
    private static Matrix DealOrinData(double[] IKInput)
    {
        Debug.Log("开始==================输入数据处理为矩阵=====================");
        #region//变量定义
        Matrix returnData = new Matrix(4, 4, "T06");//T06:{6}在{0}下的位姿
        Matrix XW = new Matrix(3, 3, "R(X,W)");//沿着x旋转的矩阵
        Matrix YP = new Matrix(3, 3, "R(Y,P)");//沿着Y旋转的矩阵
        Matrix ZR = new Matrix(3, 3, "R(Z,R)");//沿着Z旋转的矩阵
        Matrix temp = new Matrix(3, 3, "R06");//过渡
        #endregion

        #region//X的赋值
        XW.Detail[0, 0] = 1;
        XW.Detail[0, 1] = 0;
        XW.Detail[0, 2] = 0;
        XW.Detail[1, 0] = 0;
        XW.Detail[1, 1] = Math.Cos(IKInput[3] / 180D * Math.PI);//度对应的数
        XW.Detail[1, 2] = -Math.Sin(IKInput[3] / 180D * Math.PI);
        XW.Detail[2, 0] = 0;
        XW.Detail[2, 1] = Math.Sin(IKInput[3] / 180D * Math.PI);
        XW.Detail[2, 2] = Math.Cos(IKInput[3] / 180D * Math.PI);
        #endregion

        #region//Y的赋值
        YP.Detail[0, 0] = Math.Cos(IKInput[4] / 180D * Math.PI);
        YP.Detail[0, 1] = 0;
        YP.Detail[0, 2] = Math.Sin(IKInput[4] / 180D * Math.PI);
        YP.Detail[1, 0] = 0;
        YP.Detail[1, 1] = 1;
        YP.Detail[1, 2] = 0;
        YP.Detail[2, 0] = -Math.Sin(IKInput[4] / 180D * Math.PI);
        YP.Detail[2, 1] = 0;
        YP.Detail[2, 2] = Math.Cos(IKInput[4] / 180D * Math.PI);
        #endregion

        #region//Z的赋值
        ZR.Detail[0, 0] = Math.Cos(IKInput[5] / 180D * Math.PI);
        ZR.Detail[0, 1] = -Math.Sin(IKInput[5] / 180D * Math.PI);
        ZR.Detail[0, 2] = 0;
        ZR.Detail[1, 0] = Math.Sin(IKInput[5] / 180D * Math.PI);
        ZR.Detail[1, 1] = Math.Cos(IKInput[5] / 180D * Math.PI);
        ZR.Detail[1, 2] = 0;
        ZR.Detail[2, 0] = 0;
        ZR.Detail[2, 1] = 0;
        ZR.Detail[2, 2] = 1;
        #endregion

        #region//数据精度整理
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (Math.Abs(XW.Detail[i, j]) < 0.0000001)
                {
                    XW.Detail[i, j] = 0;
                }
                if (Math.Abs(YP.Detail[i, j]) < 0.0000001)
                {
                    YP.Detail[i, j] = 0;
                }
                if (Math.Abs(ZR.Detail[i, j]) < 0.0000001)
                {
                    ZR.Detail[i, j] = 0;
                }
            }
        }
        #endregion

        #region//XYZ旋转矩阵数据计算打印
        //for (int i = 0; i < 3; i++)
        //{
        //    Debug.Log("XW" + i + ": " + XW.Detail[i, 0] + " " + XW.Detail[i, 1] + " " + XW.Detail[i, 2]);
        //    Debug.Log("YP" + i + ": " + YP.Detail[i, 0] + " " + YP.Detail[i, 1] + " " + YP.Detail[i, 2]);
        //    Debug.Log("ZR" + i + ": " + ZR.Detail[i, 0] + " " + ZR.Detail[i, 1] + " " + ZR.Detail[i, 2]);
        //}
        #endregion

        temp = MatrixOperator.MatrixMulti(MatrixOperator.MatrixMulti(ZR, YP), XW);

        #region//赋值：T其余数值
        for (int i = 0; i < returnData.getM - 1; i++)
        {
            for (int j = 0; j < returnData.getN - 1; j++)
            {
                returnData.Detail[i, j] = temp.Detail[i, j];
            }
        }
        returnData.Detail[0, 3] = IKInput[0];
        returnData.Detail[1, 3] = IKInput[1];
        returnData.Detail[2, 3] = IKInput[2];
        returnData.Detail[3, 0] = 0;
        returnData.Detail[3, 1] = 0;
        returnData.Detail[3, 2] = 0;
        returnData.Detail[3, 3] = 1;
        #endregion

        DealOrinData2(ref returnData);

        #region//T的打印
        for (int i = 0; i < returnData.getM; i++)
        {
            Debug.Log("第" + i + "行: " + returnData.Detail[i, 0] + " " + returnData.Detail[i, 1] + " "
                                            + returnData.Detail[i, 2] + " " + returnData.Detail[i, 3]);
        }
        #endregion
        Debug.Log("结束==================输入数据处理为矩阵=====================");
        return returnData;
    }

    /// <summary>
    /// 输入数据的再次处理
    /// </summary>
    /// <param name="returnData"></param>
    private static void DealOrinData2(ref Matrix returnData)
    {
        Matrix TempMatrix = new Matrix(4, 4, "TempMatrix");
        #region//J5与J6点
        TempMatrix.Detail[0, 0] = 1;
        TempMatrix.Detail[0, 1] = 0;
        TempMatrix.Detail[0, 2] = 0;
        TempMatrix.Detail[0, 3] = 0;
        TempMatrix.Detail[1, 0] = 0;
        TempMatrix.Detail[1, 1] = 1;
        TempMatrix.Detail[1, 2] = 0;
        TempMatrix.Detail[1, 3] = 0;
        TempMatrix.Detail[2, 0] = 0;
        TempMatrix.Detail[2, 1] = 0;
        TempMatrix.Detail[2, 2] = 1;
        TempMatrix.Detail[2, 3] = -215;
        TempMatrix.Detail[3, 0] = 0;
        TempMatrix.Detail[3, 1] = 0;
        TempMatrix.Detail[3, 2] = 0;
        TempMatrix.Detail[3, 3] = 1;
        #endregion
        returnData = MatrixOperator.MatrixMulti(returnData, MatrixOperator.MatrixInvByCom(TempMatrix));
    }

    /// <summary>
    /// 计算theta1
    /// </summary>
    /// <param name="T06"></param>
    /// <param name="theta"></param>
    private static void CalculateTheta1(Matrix T06, ref double[,] theta)
    {
        Debug.Log("============================计算theta1============================");
        for (int i = 0; i < (theta.GetLength(0) / 2); i++)
        {
            theta[i, 0] = Math.Atan2(T06.Detail[1, 3], T06.Detail[0, 3]) * 180D / Math.PI;//py,px
            if (Math.Abs(theta[i, 0]) < 0.001)
            {
                theta[i, 0] = 0;
            }
        }
        for (int i = theta.GetLength(0) / 2; i < theta.GetLength(0); i++)
        {
            theta[i, 0] = Math.Atan2(-T06.Detail[1, 3], -T06.Detail[0, 3]) * 180D / Math.PI;//-py,-px
            if (Math.Abs(theta[i, 0]) < 0.001)
            {
                theta[i, 0] = 0;
            }
        }
        Debug.Log("============================结束theta1============================");
    }

    /// <summary>
    /// 计算theta3
    /// </summary>
    /// <param name="theta1"></param>
    /// <param name="PX"></param>
    /// <param name="PY"></param>
    /// <param name="PZ"></param>
    /// <param name="PValue"></param>
    /// <param name="i"></param>
    /// <param name="theta"></param>
    private static void CalculateTheta3(double theta1, double PX, double PY, double PZ, double[] PValue, int i, ref double[,] theta)
    {
        Debug.Log("============================计算theta3============================");
        double M;
        if (Math.Abs(Math.Sin(theta1 / 180D * Math.PI)) < 0.000001)
        {
            M = PX / Math.Cos(theta1 / 180D * Math.PI);
        }
        else
        {
            M = PY / Math.Sin(theta1 / 180D * Math.PI);
        }

        double N = PZ - PValue[0];
        double Q = (Math.Pow((M - PValue[1]), 2) + Math.Pow(N, 2) - Math.Pow(PValue[2], 2) - Math.Pow(PValue[3], 2) - Math.Pow(PValue[4], 2)) / (2 * PValue[2]);
        double Rou = Math.Sqrt(Math.Pow(PValue[3], 2) + Math.Pow(PValue[4], 2));

        if (1 - (Math.Pow(Q, 2) / Math.Pow(Rou, 2)) < 0)
        {
            Debug.Log("Theta3出现了奇异");
            if (Q * Rou > 0)
            {
                Q = Rou;
                Debug.Log("Q=Rou；atan(1,0)");
            }
            else if (Q * Rou < 0)
            {
                Q = -Rou;
                Debug.Log("Q=-Rou；atan(-1,0)");
            }
        }

        if (Math.Abs(Q / Rou) < 0.001)
        {
            Q = Rou;
        }
        if (Math.Abs(Q + Rou) < 0.001)
        {
            Q = -Rou;
        }

        if (i == 0 || i == 1 || i == 4 || i == 5)
        {
            theta[i, 2] = (Math.Atan2(Q / Rou, Math.Sqrt(1 - (Math.Pow(Q, 2) / Math.Pow(Rou, 2)))) - Math.Atan2(PValue[3], PValue[4])) / Math.PI * 180;
            if (Math.Abs(theta[i, 2]) < 0.001)
            {
                theta[i, 2] = 0;
            }
        }
        else
        {
            theta[i, 2] = (Math.Atan2(Q / Rou, -Math.Sqrt(1 - (Math.Pow(Q, 2) / Math.Pow(Rou, 2)))) - Math.Atan2(PValue[3], PValue[4])) / Math.PI * 180;
            if (Math.Abs(theta[i, 2]) < 0.001)
            {
                theta[i, 2] = 0;
            }
        }
        Debug.Log("============================结束theta3============================");
    }

    /// <summary>
    /// 计算theta2
    /// </summary>
    /// <param name="theta1"></param>
    /// <param name="theta3"></param>
    /// <param name="PX"></param>
    /// <param name="PY"></param>
    /// <param name="PZ"></param>
    /// <param name="PValue"></param>
    /// <param name="i"></param>
    /// <param name="theta"></param>
    private static void CalculateTheta2(double theta1, double theta3, double PX, double PY, double PZ, double[] PValue, int i, ref double[,] theta)
    {
        Debug.Log("============================计算theta2============================");
        double M;
        if (Math.Abs(Math.Sin(theta1 / 180D * Math.PI)) < 0.000001)
        {
            M = PX / Math.Cos(theta1 / 180D * Math.PI);
        }
        else
        {
            M = PY / Math.Sin(theta1 / 180D * Math.PI);
        }

        double N = PZ - PValue[0];
        double K = M - PValue[1];
        double P = PValue[3] * Math.Cos(theta3 / 180D * Math.PI) + PValue[4] * Math.Sin(theta3 / 180D * Math.PI) + PValue[2];
        double Q = -PValue[3] * Math.Sin(theta3 / 180D * Math.PI) + PValue[4] * Math.Cos(theta3 / 180D * Math.PI);

        theta[i, 1] = (Math.Atan2((K * P - N * Q), (K * Q + N * P))) / Math.PI * 180D;

        if (Math.Abs(theta[i, 1]) < 0.001)
        {
            theta[i, 1] = 0;
        }
        Debug.Log("============================结束theta2============================");
    }

    /// <summary>
    /// 计算theta5
    /// </summary>
    /// <param name="theta1"></param>
    /// <param name="theta2"></param>
    /// <param name="theta3"></param>
    /// <param name="PValue"></param>
    /// <param name="T06"></param>
    /// <param name="i"></param>
    /// <param name="theta"></param>
    private static void CalculateTheta5(double theta1, double theta2, double theta3, double[] PValue, Matrix T06, int i, ref double[,] theta)
    {
        Debug.Log("============================计算theta5============================");
        #region//前三个矩阵
        Matrix T01 = new Matrix(4, 4, "T01");
        T01.Detail[0, 0] = Math.Cos(theta1 / 180D * Math.PI);
        T01.Detail[0, 1] = -Math.Sin(theta1 / 180D * Math.PI);
        T01.Detail[0, 2] = 0;
        T01.Detail[0, 3] = 0;
        T01.Detail[1, 0] = Math.Sin(theta1 / 180D * Math.PI);
        T01.Detail[1, 1] = Math.Cos(theta1 / 180D * Math.PI);
        T01.Detail[1, 2] = 0;
        T01.Detail[1, 3] = 0;
        T01.Detail[2, 0] = 0;
        T01.Detail[2, 1] = 0;
        T01.Detail[2, 2] = 1;
        T01.Detail[2, 3] = PValue[0];
        T01.Detail[3, 0] = 0;
        T01.Detail[3, 1] = 0;
        T01.Detail[3, 2] = 0;
        T01.Detail[3, 3] = 1;

        Matrix T12 = new Matrix(4, 4, "T12");
        T12.Detail[0, 0] = Math.Sin(theta2 / 180D * Math.PI);
        T12.Detail[0, 1] = Math.Cos(theta2 / 180D * Math.PI);
        T12.Detail[0, 2] = 0;
        T12.Detail[0, 3] = PValue[1];
        T12.Detail[1, 0] = 0;
        T12.Detail[1, 1] = 0;
        T12.Detail[1, 2] = 1;
        T12.Detail[1, 3] = 0;
        T12.Detail[2, 0] = Math.Cos(theta2 / 180D * Math.PI); ;
        T12.Detail[2, 1] = -Math.Sin(theta2 / 180D * Math.PI);
        T12.Detail[2, 2] = 0;
        T12.Detail[2, 3] = 0;
        T12.Detail[3, 0] = 0;
        T12.Detail[3, 1] = 0;
        T12.Detail[3, 2] = 0;
        T12.Detail[3, 3] = 1;

        Matrix T23 = new Matrix(4, 4, "T23");
        T23.Detail[0, 0] = Math.Cos(theta3 / 180D * Math.PI);
        T23.Detail[0, 1] = -Math.Sin(theta3 / 180D * Math.PI);
        T23.Detail[0, 2] = 0;
        T23.Detail[0, 3] = PValue[3];
        T23.Detail[1, 0] = -Math.Sin(theta3 / 180D * Math.PI);
        T23.Detail[1, 1] = -Math.Cos(theta3 / 180D * Math.PI);
        T23.Detail[1, 2] = 0;
        T23.Detail[1, 3] = 0;
        T23.Detail[2, 0] = 0; ;
        T23.Detail[2, 1] = 0;
        T23.Detail[2, 2] = -1;
        T23.Detail[2, 3] = 0;
        T23.Detail[3, 0] = 0;
        T23.Detail[3, 1] = 0;
        T23.Detail[3, 2] = 0;
        T23.Detail[3, 3] = 1;

        #endregion

        Matrix iT03_T06 = MatrixOperator.MatrixMulti(MatrixOperator.MatrixInvByCom(MatrixOperator.MatrixMulti(MatrixOperator.MatrixMulti(T01, T12), T23)), T06);

        double c5 = iT03_T06.Detail[1, 2];
        double s5 = Math.Sqrt(1 - Math.Pow(c5, 2));
        if (i % 2 != 0)
        {
            s5 = -Math.Sqrt(1 - Math.Pow(c5, 2));
        }
        theta[i, 4] = Math.Atan2(s5, c5) / Math.PI * 180;

        Debug.Log("============================结束theta5============================");
    }

    /// <summary>
    /// 计算theta4、6
    /// </summary>
    /// <param name="theta1"></param>
    /// <param name="theta2"></param>
    /// <param name="theta3"></param>
    /// <param name="theta5"></param>
    /// <param name="PValue"></param>
    /// <param name="T06"></param>
    /// <param name="i"></param>
    /// <param name="theta"></param>
    private static void CalculateTheta4_6(double theta1, double theta2, double theta3, double theta5, double[] PValue, Matrix T06, int i, double[] oldtheta, ref double[,] theta)
    {
        Debug.Log("============================计算theta4_6============================");
        #region//前三个矩阵
        Matrix T01 = new Matrix(4, 4, "T01");
        T01.Detail[0, 0] = Math.Cos(theta1 / 180D * Math.PI);
        T01.Detail[0, 1] = -Math.Sin(theta1 / 180D * Math.PI);
        T01.Detail[0, 2] = 0;
        T01.Detail[0, 3] = 0;
        T01.Detail[1, 0] = Math.Sin(theta1 / 180D * Math.PI);
        T01.Detail[1, 1] = Math.Cos(theta1 / 180D * Math.PI);
        T01.Detail[1, 2] = 0;
        T01.Detail[1, 3] = 0;
        T01.Detail[2, 0] = 0;
        T01.Detail[2, 1] = 0;
        T01.Detail[2, 2] = 1;
        T01.Detail[2, 3] = PValue[0];
        T01.Detail[3, 0] = 0;
        T01.Detail[3, 1] = 0;
        T01.Detail[3, 2] = 0;
        T01.Detail[3, 3] = 1;

        Matrix T12 = new Matrix(4, 4, "T12");
        T12.Detail[0, 0] = Math.Sin(theta2 / 180D * Math.PI);
        T12.Detail[0, 1] = Math.Cos(theta2 / 180D * Math.PI);
        T12.Detail[0, 2] = 0;
        T12.Detail[0, 3] = PValue[1];
        T12.Detail[1, 0] = 0;
        T12.Detail[1, 1] = 0;
        T12.Detail[1, 2] = 1;
        T12.Detail[1, 3] = 0;
        T12.Detail[2, 0] = Math.Cos(theta2 / 180D * Math.PI); ;
        T12.Detail[2, 1] = -Math.Sin(theta2 / 180D * Math.PI);
        T12.Detail[2, 2] = 0;
        T12.Detail[2, 3] = 0;
        T12.Detail[3, 0] = 0;
        T12.Detail[3, 1] = 0;
        T12.Detail[3, 2] = 0;
        T12.Detail[3, 3] = 1;

        Matrix T23 = new Matrix(4, 4, "T23");
        T23.Detail[0, 0] = Math.Cos(theta3 / 180D * Math.PI);
        T23.Detail[0, 1] = -Math.Sin(theta3 / 180D * Math.PI);
        T23.Detail[0, 2] = 0;
        T23.Detail[0, 3] = PValue[3];
        T23.Detail[1, 0] = -Math.Sin(theta3 / 180D * Math.PI);
        T23.Detail[1, 1] = -Math.Cos(theta3 / 180D * Math.PI);
        T23.Detail[1, 2] = 0;
        T23.Detail[1, 3] = 0;
        T23.Detail[2, 0] = 0; ;
        T23.Detail[2, 1] = 0;
        T23.Detail[2, 2] = -1;
        T23.Detail[2, 3] = 0;
        T23.Detail[3, 0] = 0;
        T23.Detail[3, 1] = 0;
        T23.Detail[3, 2] = 0;
        T23.Detail[3, 3] = 1;

        #endregion
        Matrix iT03_T06 = MatrixOperator.MatrixMulti(MatrixOperator.MatrixInvByCom(MatrixOperator.MatrixMulti(MatrixOperator.MatrixMulti(T01, T12), T23)), T06);

        if (Math.Floor(Math.Abs(theta5)) == 0)
        {
            Debug.Log("theta5为0,其值为：" + theta5);
            theta[i, 3] = oldtheta[3];
            theta[i, 5] = Math.Atan2(-iT03_T06.Detail[2, 0], -iT03_T06.Detail[2, 1]) / Math.PI * 180 - oldtheta[3];
        }
        else
        {
            theta[i, 3] = Math.Atan2(iT03_T06.Detail[2, 2] * Math.Sin(theta5 / 180D * Math.PI), -iT03_T06.Detail[0, 2] * Math.Sin(theta5 / 180D * Math.PI)) / Math.PI * 180;
            theta[i, 5] = Math.Atan2(-iT03_T06.Detail[1, 1] * Math.Sin(theta5 / 180D * Math.PI), iT03_T06.Detail[1, 0] * Math.Sin(theta5 / 180D * Math.PI)) / Math.PI * 180;
        }
        Debug.Log("============================结束theta4_6============================");
    }

    /// <summary>
    /// 在求解的集合中找最合适的一组解
    /// </summary>
    /// <param name="theta">解的集合</param>
    /// <param name="oldtheta">上一个角度</param>
    /// <returns>返回最合适的解</returns>
    private static double[] BestResult(double[,] theta, double[] oldtheta)
    {
        double[] ReturnData = new double[theta.GetLength(1)];
        double[,] Biasangel = new double[theta.GetLength(0), theta.GetLength(1)];
        double[] BiasangelSum = new double[theta.GetLength(0)];

        for (int i = 0; i < theta.GetLength(0); i++)
        {
            for (int j = 0; j < theta.GetLength(1); j++)
            {
                Biasangel[i, j] = Math.Abs(theta[i, j] - oldtheta[j]);
            }
        }

        for (int i = 0; i < Biasangel.GetLength(0); i++)
        {
            BiasangelSum[i] = Biasangel[i, 0] + Biasangel[i, 1] + Biasangel[i, 2] + Biasangel[i, 3] + Biasangel[i, 4] + Biasangel[i, 5];
        }

        int index = 0;
        for (int i = 1; i < BiasangelSum.Length; i++)
        {
            if (BiasangelSum[index] > BiasangelSum[i])
            {
                index = i;
            }
        }

        for (int i = 0; i < theta.GetLength(1); i++)
        {
            ReturnData[i] = theta[index, i];
        }

        return ReturnData;
    }

}
