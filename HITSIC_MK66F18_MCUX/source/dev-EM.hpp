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

#define SampleTimes 30  //取样数组长度，一般取20-50



extern bool em_sw;



void LV_Sample(void);// ad采集函数√√
void LV_Get_Val(void); //对采集的值滤波√√
void swap(uint32_t * a, uint32_t * b);//交换函数√√
float get_EM_error(void);//误差获取，待更新
void normalization(void);//归一化
void EM_menu(void);//电磁菜单部分

#endif /* DEV_EM_HPP_ */
