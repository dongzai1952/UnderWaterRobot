#include "Imu.hpp"

void Imu::Init(UART_HandleTypeDef *huart)
{
    huart_ = huart;

    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}

void Imu::ResetYaw()
{
    //z轴置零
    uint8_t tx_data1[] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
    HAL_UART_Transmit(huart_, tx_data1, 5, 0xffff);
    HAL_Delay(200);

    uint8_t tx_data2[] = {0xff, 0xaa, 0x01, 0x04, 0x00};
    HAL_UART_Transmit(huart_, tx_data2, 5, 0xffff);
    HAL_Delay(3000);

    uint8_t tx_data3[] = {0xff, 0xaa, 0x00, 0x00, 0x00};
    HAL_UART_Transmit(huart_, tx_data3, 5, 0xffff);
    HAL_Delay(200);
}

void Imu::SetYawZero()
{
    yaw_zero_ = yaw_;
}

void Imu::Decode()
{
    if(rx_data_==0x55) rx_index_=0;  //帧头
    rx_buf_[rx_index_] = rx_data_;  // 存储接收到的字符
    rx_index_++;
    if(rx_index_ == 11)  //接收完一帧数据
    {
        uint8_t sum = 0;
        int16_t temp=0;//临时变量 运用于计算神奇的高低字符
        static int yaw_corrected_flag=0;//用于删掉最开始不稳定的数据
        //int return_flag;
        
        float pitch_temp=0;//临时变量 存放临时俯仰角        
        float roll_temp=0;
        float yaw_temp=0;
        
        float wx_temp=0;//临时变量 存放临时角速度       
        float wy_temp=0;
        float wz_temp=0;
        
        float ax_temp=0;//临时变量 存放临时加速度        
        float ay_temp=0;
        float az_temp=0;
	
	
        static float pitch_last=0;//静态变量 记录上次解算结果
        static float roll_last=0;

        //计算校验和
        for(int i=0; i<10; i++)
        {
            sum += rx_buf_[i];
        }

        if(sum != rx_buf_[10]) //校验不通过，重新接收
        {
            rx_index_ = 0;
        }
        else
        {
            //校验通过
            switch(rx_buf_[1])
            {
            case 0x51://加速度解算
                //LED0_ON();
                temp=rx_buf_[7]<<8;					
                temp|=rx_buf_[6];   //处理神奇的高低字节
                az_temp=((float)temp)/32768.0f*16.0f*9.8f; //转换	单位 m*s^-2																		
                acc_z_=az_temp;//Z轴角速度

                temp=rx_buf_[5]<<8;					
                temp|=rx_buf_[4];   //处理神奇的高低字节
                ay_temp=((float)temp)/32768.0f*16.0f*9.8f; //转换	单位 m*s^-2																			
                acc_y_=ay_temp;//Y轴角速度

                temp=rx_buf_[3]<<8;					
                temp|=rx_buf_[2];   //处理神奇的高低字节
                ax_temp=((float)temp)/32768.0f*16.0f*9.8f; //转换	单位 m*s^-2																			
                acc_x_=ax_temp;//X轴角速度
                
                Acc_Update_FLAG_=1;
                
                rx_buf_[1]=0;	
                break;
            case 0X52 ://角速度解算
                //LED1_ON();
                temp=rx_buf_[7]<<8;					
                temp|=rx_buf_[6];   //处理神奇的高低字节
                wz_temp=((float)temp)/32768*2000; //转换	单位°/S																		
                w_z_=wz_temp;//Z轴角速度

                temp=rx_buf_[5]<<8;					
                temp|=rx_buf_[4];   //处理神奇的高低字节
                wy_temp=((float)temp)/32768*2000; //转换	单位°/S																		
                w_y_=wy_temp;//Y轴角速度

                temp=rx_buf_[3]<<8;					
                temp|=rx_buf_[2];   //处理神奇的高低字节
                wx_temp=((float)temp)/32768*2000; //转换	单位°/S																		
                w_x_=wx_temp;//X轴角速度
                
                Palstance_Update_FLAG_=1;
                rx_buf_[1]=0;
                break;
            case 0X53 ://角度解算																						
                temp=rx_buf_[7]<<8;					
                temp|=rx_buf_[6];   //处理神奇的高低字节
                yaw_temp=((float)temp)/32768*180; //转换																			
                yaw_=yaw_temp;//偏航角仍然等于直接计算得出的值																					
                if(yaw_corrected_flag<50)//若最初10组参数，不取，略过
                {
                    yaw_corrected_flag++;//计数变量++
                    yaw_=yaw_;//偏航角仍然等于直接计算得出的值																		 
                }
                                                                                                                                            

                temp=rx_buf_[3]<<8;					
                temp|=rx_buf_[2];   //处理神奇的高低字节																											
                pitch_temp=((float)temp)/32768*180; //转换																												

                if((fabs(pitch_last-pitch_temp))<30)//传感器限幅
                        pitch_=pitch_temp;//若两次俯仰角之间的变化小于30度，记录为正确的俯仰角值，否则抛弃								
                else
                        {
                        //处理超限制问题
                        //若为前几次采集，直接赋值																														
                        if(yaw_corrected_flag<5)//若最初5组参数，直接取值
                                pitch_=pitch_temp;
                        else
                            { 
                                //判定是否为180度临界
                                if(pitch_temp>160)
                                    pitch_=pitch_temp;//此时情况较为复杂直接赋值
                                if(pitch_temp<-160)
                                    pitch_=pitch_temp;//此时情况较为复杂直接赋值				
                            }
                        }
                
                                                            
                temp=rx_buf_[5]<<8;					
                temp|=rx_buf_[4];   //处理神奇的高低字节																
                roll_temp=-((float)temp)/32768*180; //转换
                if((fabs(roll_last-roll_temp))<30)//传感器限幅
                        roll_=roll_temp;//若两次横滚角之间的变化小于30度，记录为正确的横滚角值，否则抛弃
                else
                    {
                        //处理超限制问题											
                        //若为前几次采集，直接赋值																														
                        if(yaw_corrected_flag<5)//若最初5组参数，直接取值
                                roll_=roll_temp;
                        else
                            { 
                                //判定是否为180度临界
                                if(pitch_temp>160)
                                    roll_=roll_temp;//此时情况较为复杂直接赋值
                                if(pitch_temp<-160)
                                    roll_=roll_temp;//此时情况较为复杂直接赋值				
                            }
                    }
                                                                                                                
                
                pitch_last=	pitch_;//记录上次解算结果
                roll_last=	roll_;//记录上次解算结果
                                                            
                Angle_Update_FLAG_=1;
        //	LED0_ON();		
                rx_buf_[1]=0;
                                    // printf("%f  %f  %f\n",AUH.Sys_Pitch_Angle,AUH.Sys_Roll_Angle,AUH.Sys_Yaw_Angle);
                break;
                default:	;
      
            }
        }
    }
    //开启串口中断
    HAL_UART_Receive_IT(huart_, &rx_data_, 1);
}