using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Matrix : MonoBehaviour
{
    // Start is called before the first frame update
    //矩阵打包成类，矩阵为m * n,直接调用

    #region//基础定义与方法
    private double[,] A;
    private string[,] B;
    private int m, n;//m行，n列
    private string name;
    public Matrix(int am, int an)
    {
        m = am;
        n = an;
        A = new double[m, n];
        B = new string[m, n];
        name = "Result";
    }
    public Matrix(int am, int an, string aname)
    {
        m = am;
        n = an;
        A = new double[m, n];
        B = new string[m, n];
        name = aname;
    }

    public int getM
    {
        get { return m; }
    }
    public int getN
    {
        get { return n; }
    }
    public double[,] Detail
    {
        get { return A; }
        set { A = value; }
    }

    public string[,] DetailB
    {
        get { return B; }
        set { B = value; }
    }

    public string Name { get => name; set => name = value; }
    #endregion

    /// <summary>
    /// 矩阵的克隆
    /// </summary>
    /// <param name="SetMatrix">给值的矩阵</param>
    public void MatrixClone(Matrix SetMatrix)
    {
        for (int i = 0; i < SetMatrix.getM; i++)
        {
            for (int j = 0; j < SetMatrix.getN; j++)
            {
                this.Detail[i,j] = SetMatrix.Detail[i,j];
                this.DetailB[i, j] = SetMatrix.DetailB[i, j];
            }
        }
    }
}


