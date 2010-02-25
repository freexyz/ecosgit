//#include <stdio.h>
#include <cyg/io/i2c.h>
#include <cyg/io/s3c2440x_i2c.h>

CYG_S3C2440X_I2C_BUS(test_iic_bus);
CYG_I2C_DEVICE(i2c_eeprom, 
               &test_iic_bus, 
               0x50, //0xa0
               0, 
               CYG_I2C_DEFAULT_DELAY);

/*  
 * delay function  
 */   
static void Delay(int time)   
{
	for (; time > 0; time--);   
} 

int main(void)
{
    cyg_uint8  RomAddr = 86;	
    cyg_uint8  WrData = 68;
    cyg_uint8  RdData[2];
    cyg_uint8  AddrData1[3] = {16, 158, 189};
    cyg_uint8  AddrData2[3] = {18, 7, 247};
    cyg_uint32 result, ErrFlag = 0;
    
    // Write 1 Byte at RomAddr
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)&RomAddr, 1, false);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    false, (cyg_uint8*)&WrData, 1, true);
    cyg_i2c_transaction_end(&i2c_eeprom);

    Delay(2000000);
    
    // Read 1 Byte at RomAddr
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)&RomAddr, 1, true);
				    
    result = cyg_i2c_transaction_rx(&i2c_eeprom,
				    true,(cyg_uint8*)&RdData[0], 1, true, true);
				    
    cyg_i2c_transaction_end(&i2c_eeprom);
    
    if(WrData != RdData[0])
      ErrFlag |= 0x1; 
           
    // Write 2 Byte at AddrData1[0]
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)AddrData1, 3, true);
    cyg_i2c_transaction_end(&i2c_eeprom);

    Delay(2000000);
    
    // Read 2 Byte at AddrData1[0]
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)&AddrData1[0], 1, true);
				    
    result = cyg_i2c_transaction_rx(&i2c_eeprom,
				    true,(cyg_uint8*)&RdData[0], 2, true, true);
				    
    cyg_i2c_transaction_end(&i2c_eeprom);    
    
    if((AddrData1[1] != RdData[0]) || (AddrData1[2] != RdData[1]))
      ErrFlag |= 0x2; 
    
    // Write 2 Byte at AddrData2[0]
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)&AddrData2[0], 1, false);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    false, (cyg_uint8*)&AddrData2[1], 1, false);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    false, (cyg_uint8*)&AddrData2[2], 1, true);
    cyg_i2c_transaction_end(&i2c_eeprom);

    Delay(2000000);
    
    // Read 2 Byte at AddrData2[0]
    cyg_i2c_transaction_begin(&i2c_eeprom);
    result = cyg_i2c_transaction_tx(&i2c_eeprom,
                                    true, (cyg_uint8*)&AddrData2[0], 1, true);
				    
    result = cyg_i2c_transaction_rx(&i2c_eeprom,
				    true,(cyg_uint8*)&RdData[0], 1, false, false);

    result = cyg_i2c_transaction_rx(&i2c_eeprom,
				    false,(cyg_uint8*)&RdData[1], 1, true, true);
				    
    cyg_i2c_transaction_end(&i2c_eeprom);     
    
    if((AddrData2[1] != RdData[0]) || (AddrData2[2] != RdData[1]))
      ErrFlag |= 0x4;    

    return 0;
}