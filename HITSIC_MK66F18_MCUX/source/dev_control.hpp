/*
 * dev_control.hpp
 *
 *  Created on: 2020年11月23日
 *      Author: VULCAN
 */

#ifndef DEV_CONTROL_HPP_
#define DEV_CONTROL_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_host.h"
#include "drv_disp_ssd1306_port.hpp"
#include "dev-EM.hpp"

//变量部分，一般除了标志位无需传输全局变量（大概）
extern float Dist;
/********************函数部分****************
 * 打√表示已完成编写
 * 打√√为已测试或已完成但无需测试
 */


/**主控制部分*/

void Int_set(void);//插入中断√√
void my_Motor(void *a);//电机中断服务函数√
void my_steer(void *a);//舵机中断服务函数√
void steerCTRL(void);
void MotorCTRL(void);

/**功能部分*/
//float SpdFix(float x);//差速拟合曲线函数√√
void del_start(void);//延时启动函数√√
void menu_CTRL(void);//控制部分菜单√
void wifi(void);//wifi传输√√

/**数学计算部分*/
//float trans_error(float error,float high,float low,float dx);//线性部分计算√√
float my_delta(float a,float b);//求差值函数（浮点）√√


#endif /* DEV_CONTROL_HPP_ */
