
#include "MS5837.h"
#include "stdio.h"
#include "i2c.h"

MS5837_data MS5837_30BA_data;
MS5837_result MS5837_30BA_result;

char MS5837_depth[64];
char MS5837_pressure[64];
char MS5837_temp[64];
extern I2C_HandleTypeDef hi2c1;
/**************************ʵ�ֺ���********************************************
*����ԭ��:    unsigned char MS5837_30BA_Crc4()
*��������:    �Ի�ȡ��Cal_C�������CRCУ��
*******************************************************************************/

unsigned char MS5837_30BA_Crc4(void)
{
	int cnt;
	int t;
	unsigned int n_rem=0;
	unsigned char n_bit;	
	unsigned char  a=0;
	unsigned char  b=0;
	unsigned short  int n_prom[8];
	
	for( t=0;t<7;t++)
{
   n_prom[t]=MS5837_30BA_data.Cal_C[t];
}	
	n_prom[0]=((n_prom[0]) & 0x0FFF);
	n_prom[7]=0;
	for (cnt = 0; cnt < 16; cnt++)
		{
	if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);	
	else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);		
	for (n_bit = 8; n_bit > 0; n_bit--)
	{
	if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
	else n_rem = (n_rem << 1);
	}

		}
	n_rem= ((n_rem >> 12) & 0x000F);	
    a=(n_rem ^ 0x00);
	   b=MS5837_30BA_data.Cal_C[0]>>12;
		if (a==b)
		{
		   return 1;
		}
		else return 0;
}


uint8_t MS5837Init(void)
{
	uint8_t i = 0;
	uint8_t cmd;
	uint8_t buffer[2];
	cmd = MS5837_30BA_ResetCommand;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
	HAL_Delay(20);
	for ( i = 0; i < 7; i++ )
		{
			cmd = MS5837_30BA_PROM_RD + i*2;
			HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
			HAL_I2C_Master_Receive( &hi2c1, MS5837_30BA_ReadCommand, buffer, 2, 10000 );
			MS5837_30BA_data.Cal_C[i] = (((uint16_t)buffer[0] << 8) | buffer[1]);
		}
		while(1)
		{
			if(!MS5837_30BA_Crc4())                                               //CRCУ��
			{
				//printf("��ȼƳ�ʼ��ʧ��\n");
			//	HAL_Delay(100);
				
			}
			else 
			{
				//printf("��ȼƳ�ʼ���ɹ�\n");
				
			//	HAL_Delay(100);
				//break;
				return 1;
			}
		}
	
}


static void MS5837Calculate(void)
{
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation	
	int32_t dT 		= 0;
	int64_t SENS 	= 0;
	int64_t OFF 	= 0;
	int32_t SENSi = 0;
	int32_t OFFi 	= 0;  
	int32_t Ti 		= 0;    
	int64_t OFF2 	= 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = MS5837_30BA_data.D2_Temp- (uint32_t)(MS5837_30BA_data.Cal_C[5]) * 256l;
	SENS = (int64_t)(MS5837_30BA_data.Cal_C[1]) * 32768l + ( (int64_t)(MS5837_30BA_data.Cal_C[3]) * dT ) / 256l;
	OFF = (int64_t)(MS5837_30BA_data.Cal_C[2]) * 65536l + ( (int64_t)(MS5837_30BA_data.Cal_C[4]) * dT ) / 128l;
	MS5837_30BA_data.Pressure = ( MS5837_30BA_data.D1_Pres * SENS / (2097152l)-OFF ) / (8192l);

	
	// Temp conversion
	MS5837_30BA_data.Temp = 2000l + (int64_t)(dT) * MS5837_30BA_data.Cal_C[6] / 8388608LL;
	
	if ( (MS5837_30BA_data.Temp / 100) < 20 )
	{
		//Low temp
		Ti = ( 3 * (int64_t)(dT) * (int64_t)(dT) ) / (8589934592LL);
		OFFi = ( 3 * ( MS5837_30BA_data.Temp - 2000 ) * ( MS5837_30BA_data.Temp - 2000 ) ) / 2;
		SENSi = ( 5 * ( MS5837_30BA_data.Temp - 2000 ) * ( MS5837_30BA_data.Temp - 2000 ) ) / 8;
			

	}
	else if ( ( MS5837_30BA_data.Temp / 100 ) >= 20 )
	{
		//High temp
		Ti = 2 * ( dT * dT ) / (137438953472LL);
		OFFi = ( 1 * ( MS5837_30BA_data.Temp - 2000) * ( MS5837_30BA_data.Temp - 2000 ) ) / 16;
		SENSi = 0;
	}

	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	MS5837_30BA_data.Temp = (MS5837_30BA_data.Temp-Ti);
	MS5837_30BA_data.Pressure = ( ( ( MS5837_30BA_data.D1_Pres * SENS2 ) / 2097152l - OFF2 ) / 8192l ) / 10;
}


void MS5837Read(void)
{
	uint8_t cmd;
	uint8_t buffer[3];
	
	// Request D1 conversion
	cmd = MS5837_30BA_D1_OSR_8192;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
	HAL_Delay(3); // Max conversion time per datasheet
	cmd = MS5837_30BA_ADC_RD;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
	HAL_I2C_Master_Receive(&hi2c1, MS5837_30BA_ReadCommand, buffer, 3, 10000 );
	MS5837_30BA_data.D1_Pres =(uint32_t)( (buffer[0] << 16) | (buffer[1] << 8) | buffer[2] );
	
	// Request D2 conversion
	cmd = MS5837_30BA_D2_OSR_8192;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
	HAL_Delay(3); // Max conversion time per datasheet
	cmd = MS5837_30BA_ADC_RD;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_30BA_WriteCommand, &cmd, 1, 10000 );
	HAL_I2C_Master_Receive(&hi2c1, MS5837_30BA_ReadCommand, buffer, 3, 10000 );
	MS5837_30BA_data.D2_Temp = (uint32_t)( (buffer[0] << 16) | (buffer[1] << 8) | buffer[2] );	
	
	MS5837Calculate();
}

float MS5837DataGet(void)
{
	MS5837Read();
	//HAL_Delay(100);
	MS5837_30BA_result.pressure = MS5837_30BA_data.Pressure * 100.0f;
	MS5837_30BA_result.temperature = MS5837_30BA_data.Temp / 100.0f;
	//MS5837_30BA_result.depth = 0.983615*(MS5837_30BA_data.Pressure-MS5837_30BA_data.Atmdsphere_Pressure);
	//printf("\t%2f\n",MS5837_30BA_result.pressure);
	return MS5837_30BA_result.pressure;
}


