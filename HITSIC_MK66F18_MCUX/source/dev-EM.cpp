/*
 * dev-EM.cpp
 *
 *  Created on: 2020年12月19日
 *      Author: VULCAN
 */

#include "dev-EM.hpp"


uint8_t channel_name[8] = {16,23,17,18,10,11,12,13};//adc通道
uint32_t LV_Temp[8][SampleTimes];
float LV[8];
float AD[8];
float AD_MAX[8]={0};//归一化用的最大值,菜单参数
float AD_nor[8];//归一化后的值（0~100）



/**调参参数*/
float loss_TH_in = 50;//进丢线阈值
float loss_TH_out = 70;//出丢线阈值
float alpha=0.2; //电感权重


/*各种判定标志位*/
bool EM_loss = false;//丢线标志位
bool em_sw = true;//赛道保护标志位
bool ring_flag = false,ring_in=0,ring_out=0,ring_turn=0,ring_RL=0;//环岛标志位
bool pd_flag = false;//坡道标志位

/**电感对*/
uint8_t g1[2]={0,1};//内横电感
uint8_t g2[2]={2,3};//丨电感
uint8_t g3[2]={4,5};//外横电感
uint8_t gm=6;

void LV_Sample(void)
{
    for (uint8_t h=0;h<8;h++)
    {
        for(uint8_t i=0;i<=SampleTimes-1;i++)
            {
             /*获取采样初值*/

               LV_Temp[h][i]=SCADC_Sample(ADC0, 0, channel_name[h]);//这里只有两个电感，所以这个只有两行

            }
    }
}

 void LV_Get_Val(void)//约0.3mS
  {
   // 有时会在0-65535(255)间跳动
    for(uint8_t i=0;i<=8;i++)
    {
    for(uint8_t j=0;j<=SampleTimes-1;j++)
      {
         if(LV_Temp[i][j]>500)//剔除毛刺信号
         {
             LV_Temp[i][j]=500;
         }
      }
   }

  //排序
    for(uint8_t k=0;k<=8;k++)
    {
      for(uint8_t i=0;i<=SampleTimes-2;i++)
      {
        for(uint8_t j=i+1;j<=SampleTimes-1;j++)
        {
          if(LV_Temp[k][i]>LV_Temp[k][j])
            swap(&LV_Temp[k][i],&LV_Temp[k][j]);
        }
      }
   }

  for(uint8_t k=0;k<=8;k++)
  {
    LV[k]=0;
    for(uint8_t i=3;i<=SampleTimes-4;i++)
    {
         LV[k]+=(float)LV_Temp[k][i];
    }
    LV[k]=LV[k]/(SampleTimes-6);
    if( LV[k] < MinLVGot )
    {
       LV[k] = MinLVGot;
    }
  }

  AD[0] = LV[0];
  AD[1] = LV[1];
  AD[2] = LV[2];
  AD[3] = LV[3];
  AD[4] = LV[4];
  AD[5] = LV[5];
  AD[6] = LV[6];
  AD[7] = LV[7];



//保护
//  if(AD[0]<20&&AD[6]<20)
//      em_sw=0;
}
 void swap(uint32_t *a,uint32_t *b)
    {

    uint32_t temp=*a;
    *a=*b;
    *b=temp;
    }
 float get_EM_error(void)
{
    float a,b,c;
//    if(ring_turn&&(!ring_out))//环岛内
//    {
//        if(ring_RL)
//            AD_nor[g1[0]]=0.2*AD_nor[g1[0]];
//        if(!ring_RL)
//            AD_nor[g1[1]]=0.2*AD_nor[g1[1]];
//    }
    //a=(float)(AD_nor[6]-AD_nor[0])/(AD_nor[0]*AD_nor[6]+1);
    a=10*30*(AD_nor[1]-AD_nor[6])/((AD_nor[1]+AD_nor[6]+AD_nor[0])*AD_nor[0]);//三电感
    b=10*30*(AD_nor[2]-AD_nor[4])/((AD_nor[2]+AD_nor[4]+AD_nor[0])*AD_nor[0]);
    c=(1-alpha)*a+alpha*b;


    return -c;
}


/******归一化**********/
 void normalization(void)
{
 for (uint8_t i=0;i<8;i++)
     AD_nor[i]=100*AD[i]/30;

}


/**丢线判定*/
 void EM_loss_(void)
 {
     if(!EM_loss)
     {
         if (AD_nor[6]+AD_nor[1]<loss_TH_in)
         EM_loss=true;
     }

     if(EM_loss)
     {
         if (AD_nor[6]+AD_nor[1]>loss_TH_out)
         EM_loss=false;
     }
 }
 /************环岛判定*************
  * 状态图
  * 正常情况-----(竖电感大于某个值）---->[有环岛flag=1]---(横电感大于某个值）--->[入环岛in=1]-->
  * ----(路程大于2m且)------->[出环岛标记out=1]-----
  * ----->[环岛内转弯识别]<------(横电感是否大于某个值）------>[正常识别]
  *
  *
  */


 void EM_ring_(void)
 {
     if(!ring_flag)//环岛判定
     {
         //if(AD_nor[g2[0]]+AD_nor[g2[1]]>ring_flag_TH)
         ring_flag=1;
         if(AD_nor[g2[0]]>AD_nor[g2[1]])
             ring_RL=0;
         if(AD_nor[g2[0]]>AD_nor[g2[1]])
             ring_RL=1;
     }
     else if(!ring_in)//入环岛判定
     {
         //if(AD_nor[g1[0]]+AD_nor[g1[1]]>ring_in_TH)
         ring_in=1;
     }
     else if(!ring_out)//出环
     {
         //if(Dist>(2m路程)&&(AD_nor[g2[0]]+AD_nor[g2[1]]>ring_flag_TH))
         ring_out=1;
         Dist=0;
     }

     if(ring_out)//出环岛后清零
     {
         if(Dist>1000)
             EM_ring_init();
     }


     if(ring_in&&(!ring_out))//环岛内寻迹转向
     {
         if(!ring_turn)
         {
             //if(AD_nor[g1[0]]+AD_nor[g1[1]]>ring_turn_TH)
             //ring_turn=1;
         }
         else
         {
             //if(AD_nor[g1[0]]+AD_nor[g1[1]]<ring_turn_TH)
             //ring_turn=0;
         }
     }
 }

 void EM_ring_init(void)
 {
     ring_flag = 0;
     ring_in=0;
     ring_out=0;
     ring_turn=0;
     Dist=0;
 }


 void EM_menu(void)
 {
     menu_list_t *EM_CTRL;
     EM_CTRL = MENU_ListConstruct("AD", 10, menu_menuRoot);
     MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,EM_CTRL, "AD", 0, 0));
     MENU_ListInsert(EM_CTRL, MENU_ItemConstruct(varfType, &loss_TH_in, "LOSS_TH_in", 39, menuItem_data_global));
     MENU_ListInsert(EM_CTRL, MENU_ItemConstruct(varfType, &loss_TH_out, "LOSS_TH_out", 40, menuItem_data_global));




     menu_list_t *AD_sub;
     AD_sub = MENU_ListConstruct("AD", 10, menu_menuRoot);
     MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,AD_sub, "AD", 0, 0));

     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[0], "AD[0]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[1], "AD[1]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[2], "AD[2]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[3], "AD[3]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[4], "AD[4]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[5], "AD[5]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[6], "AD[6]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[7], "AD[7]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     //MENU_ListInsert(AD_sub, MENU_ItemConstruct(varfType, &AD[8], "AD[8]", 0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));


     menu_list_t *ADm_sub;
     ADm_sub = MENU_ListConstruct("ADm", 10, menu_menuRoot);
     MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,ADm_sub, "ADm", 0, 0));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[0], "AD_MAX[0]", 30, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[1], "AD_MAX[1]", 31, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[2], "AD_MAX[2]", 32, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[3], "AD_MAX[3]", 33, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[4], "AD_MAX[4]", 34, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[5], "AD_MAX[5]", 35, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[6], "AD_MAX[6]", 36, menuItem_data_global));
     MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[7], "AD_MAX[7]", 37, menuItem_data_global));
    // MENU_ListInsert(ADm_sub, MENU_ItemConstruct(varfType, &AD_MAX[8], "AD_MAX[8]", 58, menuItem_data_global));

     menu_list_t *ADnor_sub;
     ADnor_sub = MENU_ListConstruct("ADnor", 10, menu_menuRoot);
     MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType,ADnor_sub, "ADnor", 0, 0));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[0], "AD_nor[0]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[1], "AD_nor[1]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[2], "AD_nor[2]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[3], "AD_nor[3]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[4], "AD_nor[4]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[5], "AD_nor[5]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[6], "AD_nor[6]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
     MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[7], "AD_nor[7]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));
    // MENU_ListInsert(ADnor_sub, MENU_ItemConstruct(varfType, &AD_nor[8], "AD_nor[8]",  0, menuItem_data_ROFlag | menuItem_data_NoSave | menuItem_data_NoLoad));






 }
