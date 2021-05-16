using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 控制机器人运动的函数
/// 前提挂载本函数到机器人的robot上（也就是各个关节的总父物体）、给每个public变量赋予对应的关节物体
/// 在其他函数中调试用时，直接调用AllOperate函数，并传入对应参数即可
/// </summary>
public class Robot_Roate : MonoBehaviour
{
    #region//机器人的六个关节
    public GameObject j1;
    public GameObject j2;
    public GameObject j3;
    public GameObject j4;
    public GameObject j5;
    public GameObject j6;
    #endregion

    private List<float[]> _deviatetheta = new List<float[]>();//存放所有相邻角度之间的差值
    private int num_deviatetheta = 0;//记录差值的个数
    private float _sumcount = 100;//走一段路总共所需时间总值（可以自己设置）
    private float _count = 0;//每一个update的计数（总值的计数）

    /// <summary>
    /// 使得机械人模型到达初始位置，具体根据自己导入到unity中的机器人初始位置设定
    /// </summary>
    void Start()
    {
        j1.transform.localRotation = Quaternion.Euler(0, 0, 0);
        j2.transform.localRotation = Quaternion.Euler(-90, 0, 0);
        j3.transform.localRotation = Quaternion.Euler(0, 180, -90);
        j4.transform.localRotation = Quaternion.Euler(-90, 0, -180);
        j5.transform.localRotation = Quaternion.Euler(-90, 0, 90);
        j6.transform.localRotation = Quaternion.Euler(0, -90, 90);
    }

    /// <summary>
    /// 主函数，用于控制机器人运动的函数，你需要输入逆运动学求解的所有结果，每一组解存放为float[]，所有的解需要存放在List<float[]>当中
    /// </summary>
    /// <param name="theta">输入逆运动学求解的所有结果</param>
    public void AllOperat(List<float[]> theta)
    {
        //1、计算所有相邻两角度差值
        AngleDeviatio(theta);

        //2、开始动
        InvokeRepeating("RoateAngel", 2, 0.05f);

    }

    /// <summary>
    /// 旋转角度函数，使用在invokerepeating函数中
    /// </summary>
    private void RoateAngel()
    {
        //当差值组的记录——是否所有的差值组都使用完毕
        if (num_deviatetheta != _deviatetheta.Count)
        {
            _count += 1;//时间的记录
            if (_count <= _sumcount)//总时间
            {
                //每个角度需要转动的差值
                j1.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][0] / _sumcount));
                j2.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][1] / _sumcount));
                j3.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][2] / _sumcount));
                j4.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][3] / _sumcount));
                j5.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][4] / _sumcount));
                j6.transform.Rotate(new Vector3(0, 0, _deviatetheta[num_deviatetheta][5] / _sumcount));
            }
            else
            {
                //一个差值组使用完毕，初始化时间，差值组记录加一
                _count = 0;
                num_deviatetheta++;
            }
        }
        else
        {
            //所有差值组使用完毕，初始化参数，并使得invokerepeating停止
            _count = 0;
            _deviatetheta.Clear();
            num_deviatetheta = 0;
            CancelInvoke("RoateAngel");
        }

    }

    /// <summary>
    /// 计算所传数据的角度差值
    /// </summary>
    /// <returns></returns>
    private void AngleDeviatio(List<float[]> theta)
    {
        for (int i = 0; i < theta.Count - 1; i++)
        {
            float[] temp = new float[6];
            for (int j = 0; j < theta[i].Length; j++)
            {
                temp[j] = theta[i + 1][j] - theta[i][j];
            }
            _deviatetheta.Add(temp);
        }

        #region//打印相差量值
        //for (int i = 0; i < _deviatetheta.Count; i++)
        //{
        //    Debug.Log("第" + i + "组的差值分别是：" + _deviatetheta[i][0] + " " + _deviatetheta[i][1] + " " + _deviatetheta[i][2] + " "
        //        + _deviatetheta[i][3] + " " + _deviatetheta[i][4] + " " + _deviatetheta[i][5] + " ");
        //}
        #endregion

    }
}
