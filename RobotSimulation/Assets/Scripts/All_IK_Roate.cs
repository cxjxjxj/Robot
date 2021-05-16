using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class All_IK_Roate : MonoBehaviour
{
    /// <summary>
    /// 包含了运动学的求逆，求逆结果对应的仿真旋转
    /// 输入数据：各个位姿
    /// </summary>
    public void AllOperate()
    {
        //1、上一课题所传的总的信息集合
        List<double[]> IKinput = new List<double[]>();//位姿存储
        List<double[]> IKAngle = new List<double[]>();

        IKinput.Add(new double[] { 1807, 0, 1970, 0, -90, 0 }); IKAngle.Add(new double[] { 0, 0, 0, 0, 0, 0 });
        IKinput.Add(new double[] { 768.3, 0, 1969.5, 0, -90, 0 }); IKAngle.Add(new double[] { 0, -54.2, 21.1, 0, -21.1, 0 });
        IKinput.Add(new double[] { 554.9, 1630, 884.4, 0, 0, -90 }); IKAngle.Add(new double[] { 71.2, 14.2, -38.1, 0, -51.9, -161.2 });

        #region//测试数据
        //IKinput.Add(new double[] { 1807, 0, 1970, 0, -90, 0 });
        //IKAngle.Add(new double[] { 0, 0, 0, 0, 0, 0 });

        //IKinput.Add(new double[] { 1921.4, 872.1, 2410.9, -142.3, 19.4, 127 });
        //IKAngle.Add(new double[] { 22, 32, 21, 39, 41, 58 });

        //IKinput.Add(new double[] { 1142.2, 1321.3, 2581.4, 71.8, 39.3, -10.1 });
        //IKAngle.Add(new double[] { 44.1, 23.7, 38.9, -50.1, -69, -92.8 });

        //IKinput.Add(new double[] { 586.2, 2189.1, 617.3, -53.9, -5.3, 136.3 });
        //IKAngle.Add(new double[] { 77, 42, -43.9, -78, 22, 144 });

        //IKinput.Add(new double[] { 594.7, 1334.5, 823.1, -23.7, 4.5, -89.9 });
        //IKAngle.Add(new double[] { 62.7, 4.9, -43.7, -27.6, -57.1, -134.3 });
        #endregion

        List<double[]> tempangel = new List<double[]>();//double型角度存储
        List<float[]> Angel = new List<float[]>();//float型角度存储

        //2、逆运动学求解——把位姿转换为double型的角度
        //for (int i = 0; i < IKinput.Count; i++)
        //{
        //    tempangel.Add(Robot_IK.ALL(IKinput[i], IKAngle[i]));
        //}

        for (int i = 0; i < IKinput.Count; i++)
        {
            if (i == 0)
            {
                //tempangel.Add(Test_BestResult.ALL(IKinput[i], new double[] { 0, 0, 0, 0, 0, 0 }));
                tempangel.Add(Robot_IK.ALL(IKinput[i], new double[] { 0, 0, 0, 0, 0, 0 }));
            }
            else
            {
                //tempangel.Add(Test_BestResult.ALL(IKinput[i], tempangel[tempangel.Count-1]));
                tempangel.Add(Robot_IK.ALL(IKinput[i], tempangel[tempangel.Count-1]));
            }
            //tempangel.Add(Test_BestResult.ALL(IKinput[i], IKAngle[i]));
        }


        #region//把double型的位姿转换为float型的位姿
        for (int i = 0; i < tempangel.Count; i++)
        {
            float[] temp = new float[tempangel[i].Length];
            for (int j = 0; j < tempangel[i].Length; j++)
            {
                temp[j] = float.Parse(tempangel[i][j].ToString());
            }
            Angel.Add(temp);
        }
        #endregion

        #region//打印要传给unity的角度值
        for (int i = 0; i < Angel.Count; i++)
        {
            Debug.Log("角度分别为：" + Angel[i][0] + " " + Angel[i][1] + " " + Angel[i][2] + " "
                + Angel[i][3] + " " + Angel[i][4] + " " + Angel[i][5]);
        }
        #endregion

        //3、角度传给unity进行运动仿真       
        GameObject.Find("robot").GetComponent<Robot_Roate>().AllOperat(Angel);
        //GameObject.Find("robot").GetComponent<TestRoate>().AllOperat();
    }
}
