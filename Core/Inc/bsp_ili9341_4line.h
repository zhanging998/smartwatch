/**
  ******************************************************************************
  * @file   bsp_ili9341_4line.h
  * @brief  2.8寸屏ILI9341驱动文件，采用4线SPI
  *          
  ******************************************************************************
  */
#ifndef __BSP_ILI9341_4LINE_H__
#define __BSP_ILI9341_4LINE_H__
//
#include "main.h"
//
typedef struct  
{										    
	uint16_t width;			//ili9341 宽度
	uint16_t height;		//ili9341 高度
	uint16_t id;				//ili9341 ID
	uint8_t	wramcmd;		//开始写gram指令
	uint8_t  setxcmd;		//设置x坐标指令
	uint8_t  setycmd;		//设置y坐标指令	 
}_ILI9341_dev; 	  
//扫描方向定义
#define L2R_U2D  0 //从左到右,从上到下
#define L2R_D2U  1 //从左到右,从下到上
#define R2L_U2D  2 //从右到左,从上到下
#define R2L_D2U  3 //从右到左,从下到上
 
#define U2D_L2R  4 //从上到下,从左到右
#define U2D_R2L  5 //从上到下,从右到左
#define D2U_L2R  6 //从下到上,从左到右
#define D2U_R2L  7 //从下到上,从右到左	 
//屏幕显示方式
typedef enum
{
  SCAN_Vertical= 0U,  //竖屏
  SCAN_Horizontal     //横屏
} Screen_ShowDIR;
//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         		 0x001F  
#define BRED             0XF81F
#define GRED 			 			 0XFFE0
#define GBLUE						 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 		 0XBC40 //棕色
#define BRRED 					 0XFC07 //棕红色
#define GRAY  					 0X8430 //灰色
//GUI颜色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色
// 
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)    
//
void ILI9341_Init(void);	//初始化
void ILI9341_Clear(uint16_t Color);	//清屏
void ILI9341_SetCursor(uint16_t Xpos, uint16_t Ypos);	//设置光标
void ILI9341_SetArea(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);	//设置显示区域
//
void ILI9341_WR_REG(uint8_t);
void ILI9341_WR_DATA(uint8_t);
void ILI9341_WriteReg(uint8_t ili9341_Reg, uint8_t ili9341_RegValue);
//
void ILI9341_WriteRAM_Prepare(void);
void ILI9341_WriteRAM(uint16_t RGB_Code);		  
void ILI9341_Display_Dir(Screen_ShowDIR ShowDIR);	//设置屏幕显示方向
//
void LCD_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);	//绘制水平放置的矩形
void LCD_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor);	//绘制一个圆，笔宽为1个像素 





//LCD重要参数集
  typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	 wramcmd;		//开始写gram指令
	uint16_t  setxcmd;		//设置x坐标指令
	uint16_t  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////	 
#define USE_HORIZONTAL  	 3//定义液晶屏顺时针旋转方向 	0-0度旋转，1-90度旋转，2-180度旋转，3-270度旋转

//////////////////////////////////////////////////////////////////////////////////	  
//定义LCD的尺寸
#define LCD_W 240
#define LCD_H 320

//TFTLCD部分外要调用的函数		   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色

#define	LCD_RS_SET	HAL_GPIO_WritePin(LCD_DCX_GPIO_Port,LCD_DCX_Pin,GPIO_PIN_SET)    //数据/命令  PB10	  
#define	LCD_RS_CLR	HAL_GPIO_WritePin(LCD_DCX_GPIO_Port,LCD_DCX_Pin,GPIO_PIN_RESET)     //数据/命令  PB10	 


void _HW_DrawPoint(uint16_t x,uint16_t y,uint16_t color);
void _HW_FillFrame(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);
void _HW_DrawLine(uint16_t _usX1,uint16_t _usY1,uint16_t _usX2,uint16_t _usY2,uint16_t _usColor);
void LCD_direction(uint8_t direction);

		
#endif /* __BSP_ILI9341_4LINE_H__ */
