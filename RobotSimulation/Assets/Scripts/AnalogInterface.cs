using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AnalogInterface : MonoBehaviour
{
    public GameObject PrefabBox;//箱子预制体
    //public GameObject PreTape;//绳子预制体
    public GameObject PrePan;//盘的预制体
    /// <summary>
    /// 模拟输入数据
    /// </summary>
    public void InPutData()
    {
        //初始化的箱子长宽高标准值如下：
        int boxLong = 600;
        int boxWidth = 400;
        int boxHigh = 500;

        //int boxLong = 1200;
        //int boxWidth = 800;
        //int boxHigh = 1000;

        //初始化的托盘长宽标准值如下：
        //int PanLong = 1200;
        //int PanWidth = 1000;

        int PanLong = 1200;
        int PanWidth = 1000;

        Debug.Log("点击了开始按钮");
        CreatOriBox(boxLong, boxWidth, boxHigh);
        CreatPan(PanLong, PanWidth);
    }


    /// <summary>
    /// 承接数据生成箱子；所给数据均为mm
    /// </summary>
    /// <param name="boxLong">箱子长</param>
    /// <param name="boxWidth">箱子宽</param>
    /// <param name="boxHigh">箱子高</param>
    private void CreatOriBox(int boxLong, int boxWidth, int boxHigh)
    {
        Debug.Log("生成箱子开始！");

        //实例化
        GameObject Box = GameObject.Instantiate(PrefabBox, new Vector3(-0.536F, -0.052F, -1.395F), new Quaternion(0, 0, 0, 0));
        Box.name = "testBox";
        //箱子大小     
        float _boxLong = ((boxLong * 1.0f) / 1000f) / 0.6f;
        float _boxWidth = ((boxWidth * 1.0f) / 1000f) / 0.4f;
        float _boxHigh = ((boxHigh * 1.0f) / 1000f) / 0.5f;
        Transform BoxScale = Box.GetComponent<Transform>();//获取组件
        BoxScale.localScale = new Vector3(_boxLong, _boxWidth, _boxHigh);

        //箱子大小检测
        //Box.AddComponent<BoxCollider>();
        //Box.AddComponent<GetPosition>();

        //生成带子
        //CreatOriTape(boxLong);
    }

    //private void CreatOriTape(int boxLong)
    //{
    //    Debug.Log("生成带子开始！");
    //    //实例化
    //    GameObject Tapes = GameObject.Instantiate(PreTape, new Vector3(1.919199f, -0.7817256f, 0.4771004f), new Quaternion(0, -1, 0, 0));
    //    Tapes.name = "testTapes";
    //    //带子大小
    //    float _boxLong = ((boxLong * 1.0f) / 1000f) / 0.6f;
    //    Transform TapesScale = Tapes.GetComponent<Transform>();//获取组件
    //    TapesScale.localScale = new Vector3(_boxLong, 1, 1);

    //    //带子大小检测
    //    Tapes.AddComponent<BoxCollider>();
    //    Tapes.AddComponent<GetPosition>();

    //}


    /// <summary>
    /// 承接数据生成托盘；所给数据为mm
    /// </summary>
    /// <param name="panLong">托盘长</param>
    /// <param name="panWidth">托盘宽</param>
    private void CreatPan(int panLong, int panWidth)
    {
        Debug.Log("生成盘开始！");

        //实例化
        GameObject Pan = GameObject.Instantiate(PrePan, new Vector3(-1.716f, -0.772f, 0.0790f), new Quaternion(0.0f, 0f, 0f, 0f));
        Pan.GetComponent<Transform>().Rotate(new Vector3(0, 90f, 180f));
        Pan.name = "testPan";
        //托盘大小     
        float _panLong = ((panLong * 1.0f) / 1000f) / 1.2f;
        float _panWidth = ((panWidth * 1.0f) / 1000f) / 1f;
        Transform BoxScale = Pan.GetComponent<Transform>();//获取组件
        BoxScale.localScale = new Vector3(_panLong, 1, _panWidth);

        //托盘大小检测
        Pan.AddComponent<BoxCollider>();
        Pan.AddComponent<GetPosition>();
    }
}
