/*
 * dev-Em.hpp
 *
 *  Created on: 2020年12月19日
 *      Author: VULCAN
 */

#ifndef DEV_EM_HPP_
#define DEV_EM_HPP_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "drv_disp_ssd1306_port.hpp"
#include "sc_adc.h"
#include "dev_control.hpp"

#define SampleTimes 25  //取样数组长度，一般取20-50
#define MinLVGot  2     //电感最小值限定


extern bool em_sw,EM_loss,ring_flag,ring_in,ring_out,ring_turn;



void LV_Sample(void);// ad采集函数√√
void LV_Get_Val(void); //对采集的值滤波√√
void normalization(void);//归一化
void EM_loss_(void);//丢线判定
void EM_ring_(void);//环岛状态判定
void EM_ring_init(void);//环岛状态初始化
float get_EM_error(void);//误差获取，待更新






void swap(uint32_t * a, uint32_t * b);//交换函数√√
void EM_menu(void);//电磁菜单部分
#endif /* DEV_EM_HPP_ */
