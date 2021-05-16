using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreatOriBox : MonoBehaviour
{
    public GameObject PrefabBox;//箱子预制体
    public GameObject PreTape;//绳子预制体

    public static CreatOriBox instance;

    private static GameObject originBox;//初始化一个箱子

    public void AddBox(int boxLong, int boxWidth, int boxHigh)
    {
        Debug.Log("生成箱子开始！");
        GameObject Box = GameObject.Instantiate(PrefabBox, new Vector3(1.919199f, 0.4771004f, 0.7817256f), Quaternion.identity);

        //Transform BoxScale = Box.GetComponent<Transform>();//获取组件
        //BoxScale.localScale = new Vector3(1, 1, 1);

    }

}
