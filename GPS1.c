#include "zf_common_headfile.h"
#include "GPS1.h"

//1.先确定坐标点
//2.利用坐标点代入角度计算函数--方向角
//3.IMU--航向角
//4.用方向角-航向角=角度误差
//存GPS坐标点--FLASH/数组

#if Double_Record_FLAG

int      Number=0;//介质变量

double   Target_point[2][150];//用于储存采集的目标点信息，用于后续的位置计算(第一层是纬度，第二层是经度)
double   Work_target_array[2][150];//用于将从flash中读取的目标点数组的数据复制过来，当这个数组被赋值时，后续的一切和Flash再无关系
double   lat; //纬度(英飞凌的FLASH只能存32位数据，所以用double类型定义的变量存到缓冲区只能转成float)
double   lot; //经度(英飞凌的FLASH只能存32位数据，所以用double类型定义的变量存到缓冲区只能转成float)

uint32   I_lat;//double类型纬度数据整数部分
float    F_lat;//double类型纬度数据小数部分(强制转换为float)(这里用float类型定义是因为当整数部分位0时，float类型可以存到后7位)

uint32   I_lot;//double类型经度数据整数部分
float    F_lot;//double类型经度数据小数部分(强制转换为float)

void GPS_Record_flash()    //将采集的点位记录到缓冲区并储存至GPS_FLASH
{
    static int NUM=0;       //采集点的次数
    {
        lat=gnss.latitude;   //纬度(原始数据用double类型变量存储,14位)
        lot=gnss.longitude;  //经度(原始数据用double类型变量存储,13位)

        I_lat=(uint32)(lat);//经过强转对lat取整得到整数部分
        F_lat=lat-I_lat; //用double类型数据-整数部分得到小数部分并强制转换为float类型

        I_lot=(uint32)(lot);//经过强转对lot取整得到整数部分
        F_lot=lot-I_lot; //用double类型数据-整数部分得到小数部分并强制转换为float类型

        ips_show_string(0, 16*0,"R:");

        {
            flash_union_buffer[Number].uint32_type=I_lat;  //将整数维度数据强制转换后储存在FLASH 操作的数据缓冲区
            ips_show_uint (50, 16*0, flash_union_buffer[Number].uint32_type, 5);  //显示整数部分
            ips_show_uint(150, 16*0, Number, 3);//缓冲区数组位数

            Number++;

            flash_union_buffer[Number].float_type=F_lat; //将浮点数维度数据强制转换后储存在FLASH 操作的数据缓冲区
            ips_show_float(50, 16*1, flash_union_buffer[Number].float_type, 3, 6);//显示小数部分
            ips_show_uint(150, 16*1, Number, 3);//缓冲区数组位数

            ips_show_float(50, 16*2, gnss.latitude, 3, 6);//显示完整坐标

        }

         Number++;//数组下标+1,切换储存经度

        {
            flash_union_buffer[Number].uint32_type=I_lot;  //将整数经度数据强制转换后储存在FLASH 操作的数据缓冲区
            ips_show_uint (50, 16*4, flash_union_buffer[Number].uint32_type, 5);  //显示整数部分
            ips_show_uint(150, 16*4, Number, 3);//缓冲区数组位数

            Number++;

            flash_union_buffer[Number].float_type=F_lot; //将浮点数经度数据强制转换后储存在FLASH 操作的数据缓冲区
            ips_show_float(50, 16*5, flash_union_buffer[Number].float_type, 3, 6);//显示小数部分
            ips_show_uint(150, 16*5, Number, 3);//缓冲区数组位数

            ips_show_float(50, 16*6, gnss.longitude, 3, 6);//显示完整坐标
        }


        Number++;//数组下标+1,切换储存纬度

                //如果我没有手动清除GPS_FLASH，则下面这段语句会自动清除GPS_FLASH，正常流程是:每次采集点位之前先清空GPS_FLASH
        if(flash_check(FLASH_SECTION_INDEX, GPS_PAGE_INDEX))                      //判断Flash是否有数据 : 有数据返回1，无数据返回0 (该函数每按按键执行一次，在执行本函数时,GPS_FLASH没有东西，所以擦除也无妨)                                                               //
        {
          flash_erase_page(FLASH_SECTION_INDEX, GPS_PAGE_INDEX);                //擦除Flash数据
        }

          flash_write_page_from_buffer(FLASH_SECTION_INDEX, GPS_PAGE_INDEX);        //将缓冲区的数据写入到指定Flash 扇区的页码

    }

    NUM++;

       if(NUM>RP_MAX)                                                               //如果采集点次数大于规定次数的最大值，则让他等于1
           {
             NUM=1;
           }

       ips_show_uint(210, 0, NUM, 3);                                              //显示已采集的点数


}


void GPS_Flash_use()//将GPS_FLASH的数据重新读回缓冲区并赋值给数组
{

    flash_buffer_clear();//清空缓冲区

    if(flash_check(FLASH_SECTION_INDEX, GPS_PAGE_INDEX))                      //判断Flash是否有数据 : 有数据返回1，无数据返回0
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, GPS_PAGE_INDEX);       //将数据从FLASH指定扇区页码放入到缓冲区

        int TG=0;//用以切换二维数组下标

        for(int data=0;data<200;data++)
          {

            Target_point[0][TG]=flash_union_buffer[data].uint32_type+flash_union_buffer[data+1].float_type; //纬(这里似乎要用double类型的数组,因为理论上是整数加上一个float类型的小数部分)

            data=data+2;

            Target_point[1][TG]=flash_union_buffer[data].uint32_type+flash_union_buffer[data+1].float_type; //经

            data=data+1;
            TG++;//下标+1

          }


    }

}


#endif